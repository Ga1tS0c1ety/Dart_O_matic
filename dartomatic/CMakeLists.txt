cmake_minimum_required(VERSION 3.16)
project(dartomatic C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(appbus_client
    src/appbus/client.c
)

add_executable(appbusd
    src/appbus/main_appbus.c
    src/appbus/broker.c
)

add_executable(appbus_pub
    src/tools/appbus_pub.c
)
target_link_libraries(appbus_pub appbus_client)

add_executable(appbus_sub
    src/tools/appbus_sub.c
)
target_link_libraries(appbus_sub appbus_client)

add_executable(bridge_fake_impact
    src/tools/bridge_fake_impact.c
)
target_link_libraries(bridge_fake_impact appbus_client)

add_executable(scoring_service
    src/services/scoring/scoring_service.c
)
target_link_libraries(scoring_service appbus_client m)

add_executable(game_service
    src/services/game/game_service.c
)
target_link_libraries(game_service appbus_client)

add_executable(ui_cli
    src/tools/ui_cli.c
)
target_link_libraries(ui_cli appbus_client pthread)

add_executable(rt_agg_test
    src/tools/rt_agg_test.c
    src/rt/aggregator.c
)

add_executable(rt_orch_sim
    src/tools/rt_orch_sim.c
    src/rt/rt_orchestrator.c
    src/rt/aggregator.c
)

add_executable(rt_main
    src/rt/main_rt.c
    src/rt/rt_orchestrator.c
    src/rt/aggregator.c
    src/rt/mpu_adapter.c
    src/rt/mpu6050_thread_adapted.c
)

target_link_libraries(rt_main
    pthread
    m
)