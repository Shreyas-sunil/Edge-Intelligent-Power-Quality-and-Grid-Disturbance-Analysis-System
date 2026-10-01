# Platform-independent firmware sources (shared by the Pico and host builds).
set(PQ_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/../src)
set(PQ_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR}/../config)

set(PQ_CORE_SOURCES
  ${PQ_SRC_DIR}/common/pq_types.cpp
  ${PQ_SRC_DIR}/common/math_util.cpp
  ${PQ_SRC_DIR}/adc/adc_driver.cpp
  ${PQ_SRC_DIR}/adc/adc_calibration.cpp
  ${PQ_SRC_DIR}/adc/waveform_generator.cpp
  ${PQ_SRC_DIR}/adc/sim_adc.cpp
  ${PQ_SRC_DIR}/adc/ads131m02_protocol.cpp
  ${PQ_SRC_DIR}/dsp/filtering.cpp
  ${PQ_SRC_DIR}/dsp/frequency.cpp
  ${PQ_SRC_DIR}/dsp/rms.cpp
  ${PQ_SRC_DIR}/dsp/fft.cpp
  ${PQ_SRC_DIR}/dsp/thd.cpp
  ${PQ_SRC_DIR}/dsp/wavelet.cpp
  ${PQ_SRC_DIR}/dsp/transient.cpp
  ${PQ_SRC_DIR}/power_quality/disturbance_detector.cpp
  ${PQ_SRC_DIR}/features/feature_extractor.cpp
  ${PQ_SRC_DIR}/fusion/decision_fusion.cpp
  ${PQ_SRC_DIR}/tinyml/classifier.cpp
  ${PQ_SRC_DIR}/tinyml/model_data.cpp
  ${PQ_SRC_DIR}/app/pq_engine.cpp
  ${PQ_SRC_DIR}/app/realtime_task.cpp
  ${PQ_SRC_DIR}/app/supervisor.cpp
  ${PQ_SRC_DIR}/app/benchmark.cpp
  ${PQ_SRC_DIR}/communication/json_writer.cpp
  ${PQ_SRC_DIR}/communication/transport.cpp
  ${PQ_SRC_DIR}/communication/protocol.cpp
  ${PQ_SRC_DIR}/outputs/control_output.cpp
  ${PQ_SRC_DIR}/outputs/alarm_output.cpp
  ${PQ_SRC_DIR}/storage/event_logger.cpp
  ${PQ_SRC_DIR}/system/config.cpp
  ${PQ_SRC_DIR}/system/log_sink.cpp
  ${PQ_SRC_DIR}/system/system_monitor.cpp)
