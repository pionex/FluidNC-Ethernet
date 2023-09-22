// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#include "W5500.h"
#include "Machine/MachineConfig.h"
#include "Channel.h"
#include "Report.h"

#include "Driver/sdspi.h"
#include "src/SettingsDefinitions.h"

#include <ETH.h>
#include "driver/spi_master.h"


W5500::W5500() {}

bool W5500::setupW5500() 
{
  WiFi.begin();
  
  tcpip_adapter_set_default_eth_handlers();

  // Initialize TCP/IP network interface
  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
  esp_netif_t *eth_netif = esp_netif_new(&cfg);

  esp_eth_mac_t *eth_mac = NULL;
  esp_eth_phy_t *eth_phy = NULL;
  
  gpio_install_isr_service(0);  // Should this be in gpio?

//   spi_bus_config_t buscfg = {
//     .mosi_io_num = 32,  // MOSI
//     .miso_io_num = 33,  // MISO
//     .sclk_io_num = 17,  // SCLK
//     .quadwp_io_num = -1,
//     .quadhd_io_num = -1,
//   };
//   ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, 1));

  spi_device_handle_t spi_handle = NULL;
  
  
  spi_device_interface_config_t devcfg = {
      .command_bits = 16,   // Address phase in W5500 SPI frame
      .address_bits = 8,    // Control phase in W5500 SPI frame
      .mode = 0,
      .clock_speed_hz = 12 * 1000 * 1000,
      .spics_io_num = _cs.getNative(Pin::Capabilities::Output | Pin::Capabilities::Native),    // SCS
      .queue_size = 20
  };
  ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle));

  /* W5500 ethernet driver uses spi driver */
  eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
  w5500_config.int_gpio_num = _interrupt.getNative(Pin::Capabilities::Input | Pin::Capabilities::Native);   // INT

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.reset_gpio_num = -1;  // todo: make this configurable

  eth_mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  if(eth_mac == NULL){
    log_e("esp_eth_mac_new_esp32 failed");
    return false;
  }

  eth_phy = esp_eth_phy_new_w5500(&phy_config);
  if(eth_phy == NULL){
    log_e("esp_eth_phy_new failed");
    return false;
  }
    
  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(eth_mac, eth_phy);
  esp_eth_handle_t eth_handle = NULL;
  ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

  uint8_t macArr[] = { 0x02, 0x00, 0x00, 0x12, 0x34, 0x56 };
  ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, macArr));

  /* Attach Ethernet driver to TCP/IP stack */
  ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

  /* Start Ethernet driver state machine */
  ESP_ERROR_CHECK(esp_eth_start(eth_handle));

  return true;
}


void W5500::init() {
    static bool init_message = true;  // used to show messages only once.
    pinnum_t    csPin;
    int         csFallback;

    if (_cs.defined()) {
        if (!config->_spi->defined()) {
            log_error("SD needs SPI defined");
        } else {
            log_info("W5500 cs_pin:" << _cs.name() << " detect:" << _interrupt.name());
            init_message = false;
        }
        _cs.setAttr(Pin::Attr::Output);
        csPin = _cs.getNative(Pin::Capabilities::Output | Pin::Capabilities::Native);
    } 
    else {
        log_debug("See http://wiki.fluidnc.com/en/config/w5500");
        return;
    }

    if (_interrupt.defined()) {
        _interrupt.setAttr(Pin::Attr::Input);
    } else {
        log_debug("See http://wiki.fluidnc.com/en/config/w5500");
        return;
    }

    setupW5500();
    //auto interrupt_pin = _interrupt.getNative(Pin::Capabilities::Input | Pin::Capabilities::Native);
}

void W5500::afterParse() {
}

W5500::~W5500() {}
