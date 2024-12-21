// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "BLESerial.h"

BLESerial* BLESerial::_instance = NULL;

BLESerial::BLESerial(BLEPeripheral *peripheral) : peripheral(peripheral) {
  this->_txCount = 0;
  this->_rxHead = this->_rxTail = 0;
  this->_flushed = 0;
  BLESerial::_instance = this;
}

void BLESerial::init() {  
  this->peripheral->addAttribute(this->_uartService);
  this->peripheral->addAttribute(this->_uartNameDescriptor);
  this->peripheral->setAdvertisedServiceUuid(this->_uartService.uuid());
  this->peripheral->addAttribute(this->_rxCharacteristic);
  this->peripheral->addAttribute(this->_rxNameDescriptor);
  this->_rxCharacteristic.setEventHandler(BLEWritten, BLESerial::_received);
  this->peripheral->addAttribute(this->_txCharacteristic);
  this->peripheral->addAttribute(this->_txNameDescriptor);
}

void BLESerial::poll() {
  if (millis() < this->_flushed + 100) {
    //this->peripheral->poll();
    return;
  } else {
    flush();
  }
}

void BLESerial::end() {
  this->_rxCharacteristic.setEventHandler(BLEWritten, NULL);
  this->_rxHead = this->_rxTail = 0;
  flush();
  this->peripheral->disconnect();
}

int BLESerial::available(void) {
  //this->peripheral->poll();
  int retval = (this->_rxHead - this->_rxTail + sizeof(this->_rxBuffer)) % sizeof(this->_rxBuffer);
  return retval;
}

int BLESerial::peek(void) {
  //this->peripheral->poll();
  if (this->_rxTail == this->_rxHead) return -1;
  uint8_t byte = this->_rxBuffer[this->_rxTail];
  return byte;
}

int BLESerial::read(void) {
  this->peripheral->poll();
  if (this->_rxTail == this->_rxHead) return -1;
  this->_rxTail = (this->_rxTail + 1) % sizeof(this->_rxBuffer);
  uint8_t byte = this->_rxBuffer[this->_rxTail];
  return byte;
}

void BLESerial::flush(void) {
  if (this->_txCount == 0) return;
  this->_txCharacteristic.setValue(this->_txBuffer, this->_txCount);
  this->_flushed = millis();
  this->_txCount = 0;
  this->peripheral->poll();

}

size_t BLESerial::write(uint8_t byte) {
  this->peripheral->poll();
  if (this->_txCharacteristic.subscribed() == false) return 0;
  this->_txBuffer[this->_txCount++] = byte;
  if (this->_txCount == sizeof(this->_txBuffer)) flush();

  return 1;
}

BLESerial::operator bool() {
  bool retval = this->peripheral->connected();
  return retval;
}

void BLESerial::_received(const uint8_t* data, size_t size) {
  for (int i = 0; i < size; i++) {
    this->_rxHead = (this->_rxHead + 1) % sizeof(this->_rxBuffer);
    this->_rxBuffer[this->_rxHead] = data[i];
  }
}

void BLESerial::_received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic) {
  BLESerial::_instance->_received(rxCharacteristic.value(), rxCharacteristic.valueLength());
}
