#include <Arduino.h>

void checkForNewCommands(float* gains, int n_gains, client) {
  // Accept new clients if none connected
  if (!client.connected()) {
    return;
  }
  
  // Check if data is available from the client
  if (client && client.available()) {
    // Look for start byte (255)
    uint8_t startByte = client.read();
    
    if (startByte == 255) {
      // 4 bytes per float32 sent
      int expectedBytes = 4*n_gains+1;
      
      // Wait for all bytes to arrive
      unsigned long startTime = millis();
      while (client.available() < expectedBytes && (millis() - startTime) < 100) {
        delay(1);
      }
      
      if (client.available() >= expectedBytes) {
        // Read all the data bytes and put them in the buffer for them to be parsed
        for (int i = 0; i < expectedBytes; i++) {
          receiveBuffer[i] = client.read();
        }
        
        // Verify checksum
        if (verifyChecksum(receiveBuffer, expectedBytes)) {
          // Parse the gains from the buffer
          parseGains(receiveBuffer, expectedBytes - 1);  // -1 because last byte is checksum
          newDataReceived = true;
        } else {
          Serial.println("Wrong checksum...");
        }
      }
    }
  }
}