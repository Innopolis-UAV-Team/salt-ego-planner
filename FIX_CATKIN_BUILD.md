# Исправления для успешной сборки с catkin build

## Проблема
При попытке собрать проект с помощью `catkin build` возникали следующие ошибки:
1. Ошибки линковки библиотеки `pose_utils`
2. Неправильные зависимости сборки между пакетами
3. Ошибки генерации ROS сообщений в `multi_map_server`

## Решение

### 1. Исправление pose_utils/CMakeLists.txt

**Файл:** `src/uav_simulator/Utils/pose_utils/CMakeLists.txt`

**Проблема:** Библиотека `pose_utils` не линковалась с зависимостями Armadillo

**Исправление:**
```cmake
# Было:
add_library(pose_utils 
   ${ARMADILLO_LIBRARIES}
   src/pose_utils.cpp)

# Стало:
add_library(pose_utils 
   src/pose_utils.cpp)

target_link_libraries(pose_utils
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
)
```

### 2. Исправление odom_visualization/package.xml

**Файл:** `src/uav_simulator/Utils/odom_visualization/package.xml`

**Проблема:** Отсутствовала зависимость от `quadrotor_msgs`, которая используется в коде

**Исправление:** Добавить зависимости в секции build_depend и run_depend:
```xml
<build_depend>quadrotor_msgs</build_depend>
<run_depend>quadrotor_msgs</run_depend>
```

### 3. Исправление odom_visualization/CMakeLists.txt

**Файл:** `src/uav_simulator/Utils/odom_visualization/CMakeLists.txt`

**Проблема:** Прямая ссылка на библиотеку `pose_utils` вызывала ошибки линковки

**Исправление:**
```cmake
# Было:
target_link_libraries(odom_visualization
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   pose_utils
)

# Стало:
target_link_libraries(odom_visualization
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
)
```

*Примечание:* `pose_utils` теперь подключается автоматически через `${catkin_LIBRARIES}`, так как объявлена в `find_package(catkin REQUIRED COMPONENTS ... pose_utils ...)`

### 4. Исправление multi_map_server/CMakeLists.txt

**Файл:** `src/uav_simulator/Utils/multi_map_server/CMakeLists.txt`

**Проблема 1:** Неправильное имя цели для зависимостей генерации сообщений

**Исправление:**
```cmake
# Было:
add_dependencies(multi_map_visualization multi_map_server_messages_cpp)

# Стало:
add_dependencies(multi_map_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

**Проблема 2:** Прямая ссылка на библиотеку `pose_utils`

**Исправление:**
```cmake
# Было:
target_link_libraries(multi_map_visualization 
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   pose_utils
)

# Стало:
target_link_libraries(multi_map_visualization 
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
)
```

## Команды для сборки

### Полная пересборка (рекомендуется):
```bash
cd /home/jetson/ws_ego_planner
catkin clean -y
catkin build -j2
source devel/setup.bash
```

### Обычная сборка:
```bash
cd /home/jetson/ws_ego_planner
catkin build
source devel/setup.bash
```

## Результат

После применения всех исправлений все 18 пакетов успешно собираются:
- cmake_utils
- map_generator
- mockamap
- plan_env
- pose_utils
- path_searching
- quadrotor_msgs
- bspline_opt
- traj_utils
- local_sensing_node
- multi_map_server
- odom_visualization
- rviz_plugins
- so3_control
- uav_utils
- waypoint_generator
- ego_planner
- so3_quadrotor_simulator

## Важные замечания

1. **Использование catkin вместо catkin_make:** Проект теперь собирается с помощью `catkin build`, что обеспечивает изолированную сборку каждого пакета

2. **Линковка библиотек через catkin:** При использовании catkin build библиотеки из других пакетов workspace должны подключаться через `${catkin_LIBRARIES}`, а не напрямую по имени

3. **Зависимости в package.xml:** Все зависимости, используемые в CMakeLists.txt, должны быть объявлены в package.xml

4. **Sourcing setup файлов:** После сборки обязательно выполнить `source devel/setup.bash` для обновления переменных окружения
