#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "list.h"

// #include "esp_console.h"
// #include "argtable3/argtable3.h"
// #include "cmd_decl.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/event_groups.h"
// #include "esp_wifi.h"
// #include "esp_netif.h"
// #include "esp_event.h"
// #include "cmd_wifi.h"

#define TAG "LIST"

static scan_results_list_t* scan_list = NULL;

static bool compare_bda(esp_bd_addr_t bda_src, esp_bd_addr_t bda_dest) {

  int idx = 0;
  while (idx < ESP_BD_ADDR_LEN) {
    if (bda_src[idx] != bda_dest[idx]) {
      return false;
    }
    idx++;
  }

  return true;

}

void add_scan_rest_to_list(struct ble_scan_result_evt_param* scan_rst, uint8_t* dev_name, uint8_t dev_len) {

  if (scan_rst == NULL) {
    ESP_LOGE(TAG, "%s: Empty scan result \n", __func__);
    return;
  }

  // search based on bda
  if (scan_list == NULL) {
    ESP_LOGI(TAG, "Adding first item %s: %x\n", dev_name, scan_rst->bda[0]);

    esp_log_buffer_hex(TAG, scan_rst->bda, 6);
    ESP_LOGI(TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_rst->adv_data_len, scan_rst->scan_rsp_len);
    ESP_LOGI(TAG, "searched Device Name Len %d", dev_len);
    esp_log_buffer_char(TAG, dev_name, dev_len);
    ESP_LOGI(TAG, "\n");

    scan_list = (scan_results_list_t*)malloc(sizeof(scan_results_list_t));
    scan_list->data = *scan_rst;
    strcpy(scan_list->dev_name, (char*)dev_name);
    scan_list->pNext = NULL;
  }
  else {

    // Search if the item exists or not yet
    scan_results_list_t* pHead = scan_list;
    scan_results_list_t* pPrev = scan_list;
    while (pHead != NULL) {
      // ESP_LOGI(TAG, "Traversing item in the list %x; Comparing with %x\n", scan_rst->bda[0], pHead->data.bda[0]);
      if (compare_bda(pHead->data.bda, scan_rst->bda)) {
        // Skip
        break;
      }
      pPrev = pHead;
      pHead = pHead->pNext;
    }

    if (pHead == NULL) {
      // Add the new item to the list
      scan_results_list_t* new_item;

      new_item = (scan_results_list_t*)malloc(sizeof(scan_results_list_t));
      new_item->data = *scan_rst;
      strcpy(new_item->dev_name, (char*)dev_name);
      new_item->pNext = NULL;

      esp_log_buffer_hex(TAG, scan_rst->bda, 6);
      ESP_LOGI(TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_rst->adv_data_len, scan_rst->scan_rsp_len);
      ESP_LOGI(TAG, "searched Device Name Len %d", dev_len);
      esp_log_buffer_char(TAG, dev_name, dev_len);
      ESP_LOGI(TAG, "\n");

      pPrev->pNext = new_item;
    }
  }
}

void display_scan_results() {

  scan_results_list_t* pHead = scan_list;
  int idx = 0;

  printf("Displaying scan results\n");
  while (pHead != NULL) {

    printf("[%d] %s\n", idx, pHead->dev_name);

    idx++;
    pHead = pHead->pNext;
  }
}

void find_device_by_index(uint8_t idx, struct ble_scan_result_evt_param** result) {

  int j = 0;
  scan_results_list_t* pHead = scan_list;

  *result = NULL;

  while (pHead != NULL) {
    ESP_LOGI(TAG, "Searching %d %d", j, idx);
    if (j == idx) {
      *result = &(pHead->data);
      ESP_LOGI(TAG, "Found %d %d", j, idx);
      break;
    }
    pHead = pHead->pNext;
    j++;
  }

  if (*result != NULL) {
    ESP_LOGI(TAG, "Found %x", (*result)->bda[0]);
  }

}