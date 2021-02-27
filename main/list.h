#pragma once

#include "esp_gap_ble_api.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct scan_results_list {
    struct ble_scan_result_evt_param data;
    char dev_name[255];
    struct scan_results_list* pNext;
  } scan_results_list_t;

  void add_scan_rest_to_list(struct ble_scan_result_evt_param* scan_rst, uint8_t* dev_name, uint8_t dev_len);
  void display_scan_results();
  void find_device_by_index(uint8_t idx, struct ble_scan_result_evt_param** result);


#ifdef __cplusplus
}
#endif

