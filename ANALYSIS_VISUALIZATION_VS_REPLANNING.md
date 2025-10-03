# Анализ: Визуализация vs Реальное Перепланирование

## 🎯 ВЫВОДЫ ПОСЛЕ АНАЛИЗА КОДА

### ✅ **ДА, ДРОН ЛЕТИТ ПРАВИЛЬНО И ПЕРЕПЛАНИРУЕТ МАРШРУТ!**

Вы абсолютно правы! Алгоритм работает корректно, а визуализация отстаёт.

---

## 📊 КАК РАБОТАЕТ ПЕРЕПЛАНИРОВАНИЕ

### 1. **Триггеры перепланирования в EXEC_TRAJ** (ego_replan_fsm.cpp, строки 288-318)

```cpp
case EXEC_TRAJ:
{
  LocalTrajData *info = &planner_manager_->local_data_;
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - info->start_time_).toSec();
  
  // Вычисляет ПЛАНОВУЮ позицию по времени
  Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

  // УСЛОВИЕ 1: Траектория закончилась
  if (t_cur > info->duration_ - 1e-2) {
    have_target_ = false;
    changeFSMExecState(WAIT_TARGET, "FSM");
    return;
  }
  
  // УСЛОВИЕ 2: Близко к цели (no_replan_thresh = 1.0м)
  else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
    return;  // НЕ перепланировать
  }
  
  // УСЛОВИЕ 3: Недалеко от старта (replan_thresh = 1.5м)
  else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
    return;  // НЕ перепланировать
  }
  
  // УСЛОВИЕ 4: Достаточно далеко от старта И далеко от цели
  else {
    changeFSMExecState(REPLAN_TRAJ, "FSM");  // ✅ ПЕРЕПЛАНИРОВАТЬ!
  }
}
```

### 🔑 **КЛЮЧЕВОЙ МОМЕНТ:**

Проверка использует **ПЛАНОВУЮ** позицию `pos = info->position_traj_.evaluateDeBoorT(t_cur)`, а **НЕ** реальную одометрию `odom_pos_`!

---

## 2. **Как происходит перепланирование** (ego_replan_fsm.cpp, строки 346-373)

```cpp
bool EGOReplanFSM::planFromCurrentTraj()
{
  LocalTrajData *info = &planner_manager_->local_data_;
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - info->start_time_).toSec();

  // ⚠️ БЕРЁТ СТАРТОВУЮ ТОЧКУ ИЗ ТРАЕКТОРИИ, А НЕ ИЗ ОДОМЕТРИИ!
  start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
  start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
  start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

  bool success = callReboundReplan(false, false);
  // ... попытки перепланирования
  
  return success;
}
```

### 📌 **Важно:**
При перепланировании стартовая точка берётся из **текущей траектории**, а не из **реальной одометрии**!

---

## 3. **Публикация траектории** (ego_replan_fsm.cpp, строки 418-460)

```cpp
bool EGOReplanFSM::callReboundReplan(...)
{
  getLocalTarget();
  
  bool plan_success = planner_manager_->reboundReplan(
    start_pt_, start_vel_, start_acc_,      // Из траектории!
    local_target_pt_, local_target_vel_,
    ...
  );

  if (plan_success)
  {
    auto info = &planner_manager_->local_data_;
    
    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.start_time = info->start_time_;  // ⏰ Время старта траектории
    
    // Контрольные точки B-сплайна
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.cols(); ++i) {
      // ... добавление точек
    }
    
    bspline_pub_.publish(bspline);  // 📡 Публикация
    
    // 🎨 ВИЗУАЛИЗАЦИЯ
    visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
  }
}
```

---

## 4. **Визуализация** (planning_visualization.cpp, строки 186-201)

```cpp
void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
{
  if (optimal_list_pub.getNumSubscribers() == 0) {
    return;  // ❌ Если никто не подписан, НЕ публикуется!
  }

  vector<Eigen::Vector3d> list;
  for (int i = 0; i < optimal_pts.cols(); i++) {
    Eigen::Vector3d pt = optimal_pts.col(i).transpose();
    list.push_back(pt);
  }
  
  Eigen::Vector4d color(1, 0, 0, 1);  // Красный цвет
  displayMarkerList(optimal_list_pub, list, 0.15, color, id);
}
```

### 🎨 **Что визуализируется:**
- **Контрольные точки B-сплайна** (не вся траектория!)
- Публикуется в топик `/ego_planner_node/optimal_list`
- Красным цветом, размер маркера 0.15

---

## 🔴 ПРОБЛЕМА С ВИЗУАЛИЗАЦИЕЙ

### Возможные причины расхождения:

1. **Контрольные точки vs Траектория**
   - Визуализируются только контрольные точки B-сплайна
   - Реальная траектория между ними интерполируется
   - Контрольных точек мало → визуализация выглядит "грубой"

2. **Время обновления**
   - Перепланирование происходит когда `(info->start_pos_ - pos).norm() >= 1.5м`
   - Это довольно большое расстояние!
   - Визуализация обновляется только при перепланировании

3. **Отсутствие подписчика**
   - Если на топик `/ego_planner_node/optimal_list` никто не подписан
   - Визуализация **вообще не публикуется**!

4. **ID маркера всегда 0**
   - `visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);`
   - Все маркеры имеют ID=0, старые могут затираться

---

## ✅ ПОЧЕМУ ДРОН ЛЕТИТ ПРАВИЛЬНО

### 1. **traj_server публикует команды по времени**

```cpp
// traj_server.cpp
void cmdCallback(const ros::TimerEvent &e)
{
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  
  if (t_cur < traj_duration_) {
    pos = traj_[0].evaluateDeBoorT(t_cur);  // Точная позиция из B-сплайна
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
  }
  
  cmd.position = pos;
  cmd.velocity = vel;
  cmd.acceleration = acc;
  
  pos_cmd_pub.publish(cmd);  // 100 Гц!
}
```

### 2. **Контроллер отслеживает позицию**
- SO3 контроллер получает `PositionCommand`
- Сравнивает с реальной одометрией
- Вычисляет управление для минимизации ошибки

### 3. **Перепланирование адаптирует траекторию**
- Каждые ~1.5 метра траектория обновляется
- Новая траектория учитывает препятствия
- Стартовая точка берётся из "идеальной" траектории

---

## 🎯 ВЫВОД

### ✅ **Дрон летит правильно, потому что:**
1. **traj_server** публикует детализированные команды (100 Гц)
2. **Контроллер** отслеживает позицию с обратной связью
3. **Перепланирование** обновляет траекторию регулярно
4. **B-сплайн** обеспечивает плавную интерполяцию

### ❌ **Визуализация отстаёт, потому что:**
1. Показывает только **контрольные точки** (не всю траекторию)
2. Обновляется только при **перепланировании** (раз в 1.5м)
3. Не учитывает **реальную одометрию**
4. Может не публиковаться из-за отсутствия подписчиков

---

## 💡 РЕШЕНИЯ ДЛЯ УЛУЧШЕНИЯ ВИЗУАЛИЗАЦИИ

### Вариант 1: Визуализировать плотную траекторию
```cpp
// Вместо контрольных точек, сэмплировать всю траекторию
vector<Eigen::Vector3d> traj_points;
double dt = 0.1;  // шаг 0.1 секунды
for (double t = 0; t < info->duration_; t += dt) {
  traj_points.push_back(info->position_traj_.evaluateDeBoorT(t));
}
visualization_->displayOptimalList(traj_points, 0);
```

### Вариант 2: Добавить визуализацию текущей позиции
```cpp
// Показать где дрон ДОЛЖЕН быть vs где ЕСТЬ
visualization_->displayCurrentPosition(odom_pos_);
visualization_->displayPlannedPosition(info->position_traj_.evaluateDeBoorT(t_cur));
```

### Вариант 3: Увеличить частоту обновления
```cpp
// Публиковать визуализацию чаще, не только при перепланировании
// В execFSMCallback добавить:
if (exec_state_ == EXEC_TRAJ) {
  visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
}
```

### Вариант 4: Использовать уникальные ID
```cpp
// Чтобы старые траектории не затирались
static int viz_id = 0;
visualization_->displayOptimalList(info->position_traj_.get_control_points(), viz_id++);
```

---

## 📋 Нужна помощь с реализацией?

Могу помочь:
1. ✅ Улучшить визуализацию траектории
2. ✅ Добавить отладочную информацию
3. ✅ Показать реальную vs плановую позицию
4. ✅ Настроить параметры перепланирования

Какой вариант вам интересен?
