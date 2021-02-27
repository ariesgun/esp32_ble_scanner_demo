| Supported Targets | ESP32 |
| ----------------- | ----- |

ESP-IDF BLE Scanner Demo
========================

This is the demo for users to use ESP_APIs to create a GATT Client, scan and select a remove device to connect, to write value to a certain characteristic.

To test this demo, you need to run a gatt server with certain uuid and characteristics.
TODO: Provide the gatt server information.

This demo will enable gatt server's notification function once the connection is established and then the devices start exchanging data.