var hexChar = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F"];

function byteToHex(b) {
  return hexChar[(b >> 4) & 0x0f] + hexChar[b & 0x0f];
}

function hexToInt(hex) {
  var num = hex;
  if (num > 0x7F) {
    num = num - 0x100;
  }
  return num;
}

function Decoder(bytes, port) {
  if (port == 2) {
    var mac1 = "";
    for (i = 0; i < 6; i++) {
      mac1 += byteToHex(bytes[i + 1]);
      if (i < 5) {
        mac1 += ':';
      }
    }
    var rssi1 = hexToInt(bytes[0]);
    var mac2 = "";
    for (i = 0; i < 6; i++) {
      mac2 += byteToHex(bytes[i + 8]);
      if (i < 5) {
        mac2 += ':';
      }
    }
    var rssi2 = hexToInt(bytes[7]);
    var mac3 = "";
    for (i = 0; i < 6; i++) {
      mac3 += byteToHex(bytes[i + 15]);
      if (i < 5) {
        mac3 += ':';
      }
    }
    var rssi3 = hexToInt(bytes[14]);
    var temp = ((bytes[22] << 8) | bytes[23]);
    if ((bytes[22] >> 7) == 1) {
      temp = (0xFFFF ^ (temp)) + 1;
      temp = temp - temp - temp;
    }
    var press = ((bytes[24] << 8) | bytes[25]);
    var hum = (bytes[26]);
    return {
wifiAccessPoints: [
      {
"macAddress": mac1,
"signalStrength": rssi1
      },
      {
"macAddress": mac2,
"signalStrength": rssi2
      },
      {
"macAddress": mac3,
"signalStrength": rssi3
      }
      ],
battery: String(3.3 / 255) * ((4.7 + 10) / 10) * (bytes[21]),
temperature: String(temp / 100),
pressure: String(press / 10),
humidity: String(hum),
inMotion: String(bytes[27])
    };
  }

  if (port == 3) {
    return {
payload: String("GNSS data")
    };
  }

  if (port == 10) {
    return {
battery: String(3.3 / 255) * ((4.7 + 10) / 10) * (bytes[0])
    };
  }

  if (port == 199) {
    return {
payload: String("LoRa Cloud payload")
    };
  }

  if (port == 202) {
    return {
payload: String("Alc sync payload")
    };
  }

  if (port == 44) {
    return {
ledActivation: String(bytes[0]),
interval: String(bytes[1] << 8 | bytes[2]),
beacon: String(bytes[3]),
wifiActivation: String(bytes[4]),
gnssActivation: String(bytes[5]),
motionActivation: String(bytes[6]),
motionIntervalDuration: String(bytes[7]),
motionThresholdRegister: String(bytes[8]),
motionDurationRegister: String(bytes[9])
    };
  }

}
