#include <stdio.h>

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"

#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */
#define OTA_URL_SIZE 256

#define FIRMWARE_UPG_URL "http://192.168.2.106:8070/hello_world.bin"
#define OTA_RECV_TIMEOUT 5000

void ota_task(void *pvParameter);
void print_sha256(const uint8_t *image_hash, const char *label);