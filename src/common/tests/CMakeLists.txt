set(SOLAR_TEST_SUPPORT_DIR ${CMAKE_SOURCE_DIR}/src/tests/support)

add_executable(test_common_core
    ${SOLAR_TEST_SUPPORT_DIR}/test_main.cpp
    test_threadsafequeue.cpp
    test_latency_monitor.cpp
)

target_include_directories(test_common_core PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_common_core PRIVATE solar_common)
solar_enable_coverage(test_common_core)

add_test(NAME ThreadSafeQueue_FIFO_basic
         COMMAND test_common_core --run ThreadSafeQueue_FIFO_basic)
add_test(NAME ThreadSafeQueue_bounded_push_strict_rejects_when_full
         COMMAND test_common_core --run ThreadSafeQueue_bounded_push_strict_rejects_when_full)
add_test(NAME ThreadSafeQueue_bounded_push_latest_drops_oldest
         COMMAND test_common_core --run ThreadSafeQueue_bounded_push_latest_drops_oldest)
add_test(NAME ThreadSafeQueue_wait_pop_blocks_then_wakes
         COMMAND test_common_core --run ThreadSafeQueue_wait_pop_blocks_then_wakes)
add_test(NAME ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty
         COMMAND test_common_core --run ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty)

add_test(NAME LatencyMonitor_accepts_ordered_timestamps_and_prints
         COMMAND test_common_core --run LatencyMonitor_accepts_ordered_timestamps_and_prints)
add_test(NAME LatencyMonitor_handles_out_of_order_calls_without_crashing
         COMMAND test_common_core --run LatencyMonitor_handles_out_of_order_calls_without_crashing)
add_test(NAME LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing
         COMMAND test_common_core --run LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing)
