// ВАРИАНТ 2А: Адаптивное перепланирование по отклонению
// Добавить в ego_replan_fsm.cpp

// ============================================
// В начало файла (после includes):
// ============================================

// Глобальная статистика для отладки
static double max_tracking_error_seen = 0.0;
static double avg_tracking_error = 0.0;
static int tracking_error_samples = 0;

// ============================================
// В case EXEC_TRAJ (заменить существующий код):
// ============================================

case EXEC_TRAJ:
{
  /* determine if need to replan */
  LocalTrajData *info = &planner_manager_->local_data_;
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - info->start_time_).toSec();
  t_cur = min(info->duration_, t_cur);

  Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

  // 🎨 ДИАГНОСТИЧЕСКАЯ ВИЗУАЛИЗАЦИЯ
  visualization_->displayCurrentOdomPosition(odom_pos_, 0);
  visualization_->displayPlannedPosition(pos, 0);
  visualization_->displayDenseTrajectory(info->position_traj_, info->duration_, 0);

  // 📊 ВЫЧИСЛЕНИЕ ОТКЛОНЕНИЯ
  double tracking_error = (odom_pos_ - pos).norm();
  
  // Обновление статистики (для отладки)
  tracking_error_samples++;
  avg_tracking_error = (avg_tracking_error * (tracking_error_samples - 1) + tracking_error) / tracking_error_samples;
  if (tracking_error > max_tracking_error_seen) {
    max_tracking_error_seen = tracking_error;
  }
  
  // Логировать каждую секунду
  static ros::Time last_log_time = ros::Time::now();
  if ((time_now - last_log_time).toSec() > 1.0) {
    ROS_INFO("Tracking error: current=%.3fm, avg=%.3fm, max=%.3fm", 
             tracking_error, avg_tracking_error, max_tracking_error_seen);
    last_log_time = time_now;
  }

  // ⚠️ ПРОВЕРКА КРИТИЧЕСКОГО ОТКЛОНЕНИЯ
  // TODO: Добавить параметр в launch файл
  double max_tracking_error_threshold = 0.5;  // 50см - критическое отклонение
  
  if (tracking_error > max_tracking_error_threshold) {
    ROS_WARN("⚠️  LARGE TRACKING ERROR: %.2fm > %.2fm - EMERGENCY REPLAN!", 
             tracking_error, max_tracking_error_threshold);
    changeFSMExecState(REPLAN_TRAJ, "TRACKING_ERROR");
    
    // Сбросить статистику после экстренного перепланирования
    max_tracking_error_seen = 0.0;
    avg_tracking_error = 0.0;
    tracking_error_samples = 0;
    
    break;  // Важно выйти из switch!
  }

  /* && (end_pt_ - pos).norm() < 0.5 */
  if (t_cur > info->duration_ - 1e-2)
  {
    have_target_ = false;
    
    // Финальная статистика
    ROS_INFO("✅ Trajectory completed! Final stats: avg_error=%.3fm, max_error=%.3fm", 
             avg_tracking_error, max_tracking_error_seen);
    
    // Сброс
    max_tracking_error_seen = 0.0;
    avg_tracking_error = 0.0;
    tracking_error_samples = 0;

    changeFSMExecState(WAIT_TARGET, "FSM");
    return;
  }
  else if ((end_pt_ - pos).norm() < no_replan_thresh_)
  {
    // cout << "near end" << endl;
    return;
  }
  else if ((info->start_pos_ - pos).norm() < replan_thresh_)
  {
    // cout << "near start" << endl;
    return;
  }
  else
  {
    changeFSMExecState(REPLAN_TRAJ, "FSM");
  }
  break;
}

// ============================================
// ОПЦИОНАЛЬНО: Добавить параметр в init()
// ============================================

void EGOReplanFSM::init(ros::NodeHandle &nh)
{
  // ...existing code...
  
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  
  // 🆕 НОВЫЙ ПАРАМЕТР
  nh.param("fsm/max_tracking_error", max_tracking_error_param_, 0.5);
  
  // ...existing code...
}

// ============================================
// И в header файл (ego_replan_fsm.h):
// ============================================

// В private секцию добавить:
double max_tracking_error_param_;  // Параметр из конфига

// ============================================
// В launch файл (advanced_param.xml):
// ============================================

<!--  Добавить в секцию planning fsm: -->
<param name="fsm/max_tracking_error" value="0.5" type="double"/>

// ============================================
// ИСПОЛЬЗОВАНИЕ В КОДЕ (улучшенная версия):
// ============================================

if (tracking_error > max_tracking_error_param_) {
  ROS_WARN("⚠️  LARGE TRACKING ERROR: %.2fm > %.2fm - EMERGENCY REPLAN!", 
           tracking_error, max_tracking_error_param_);
  changeFSMExecState(REPLAN_TRAJ, "TRACKING_ERROR");
  break;
}

// ============================================
// ПРЕИМУЩЕСТВА ЭТОГО ПОДХОДА:
// ============================================

/*
✅ Перепланирование только когда ДЕЙСТВИТЕЛЬНО нужно
✅ Не увеличивает нагрузку если дрон летит хорошо
✅ Автоматически адаптируется к качеству полёта
✅ Настраивается через launch-файл без перекомпиляции
✅ Логирование для анализа
✅ Статистика для оптимизации параметров

📊 Типичные значения max_tracking_error:
- 0.2м - очень строгий (для высокоточных задач)
- 0.5м - рекомендуемый (хороший баланс)
- 1.0м - мягкий (для быстрого полёта)

⚠️  ВАЖНО:
- Не ставьте слишком маленькое значение (< 0.1м)
- Это вызовет слишком частые перепланирования
- Может привести к "дрожанию" траектории
*/
