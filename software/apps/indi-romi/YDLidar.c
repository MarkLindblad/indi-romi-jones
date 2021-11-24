/* modified YDLidar arduino library to work on nrf52 dk
*/
#include "YDLidar.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_error.h"

result_t waitResponseHeader(lidar_ans_header *header, uint32_t timeout);
// YDLidar::YDLidar()
//   : _bined_serialdev(NULL) {
//   point.distance = 0;
//   point.angle = 0;
//   point.quality = 0;
// }
#define OVERFLOW ((uint32_t)(0xFFFFFFFF/32.768))

uint32_t millis(void)
{
  return(app_timer_cnt_get() / 32.768);
}

uint32_t compareMillis(uint32_t previousMillis, uint32_t currentMillis)
{
  if(currentMillis < previousMillis) return(currentMillis + OVERFLOW + 1 - previousMillis);
  return(currentMillis - previousMillis);
}



scanPoint point = {0,0,0, NULL};

// start the scanPoint operation
result_t startScan(bool force, uint32_t timeout) {
	result_t ans;

	lidar_ans_header response_header;

	if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
	  return ans;
	}

	if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
	  return RESULT_FAIL;
	}

	if (response_header.size < sizeof(node_info)) {
	  return RESULT_FAIL;
	}
	  
	  return RESULT_OK;
}

// wait scan data
result_t waitScanDot(uint32_t timeout) {
  int recvPos = 0;
  uint32_t startTs = millis();
  uint32_t waitTime;
  uint8_t nowPackageNum;
  node_info node = {0};
  static node_package package;
  static uint16_t package_Sample_Index = 0;
  static float IntervalSampleAngle = 0;
  static float IntervalSampleAngle_LastPackage = 0;
  static uint16_t FirstSampleAngle = 0;
  static uint16_t LastSampleAngle = 0;
  static uint16_t CheckSum = 0;

  static uint16_t CheckSumCal = 0;
  static uint16_t SampleNumlAndCTCal = 0;
  static uint16_t LastSampleAngleCal = 0;
  static bool CheckSumResult = true;
  static uint16_t Valu8Tou16 = 0;

  uint8_t *packageBuffer = (uint8_t *)&package.package_Head;
  uint8_t  package_Sample_Num = 0;
  int32_t AngleCorrectForDistance;

  int  package_recvPos = 0;
  if (package_Sample_Index == 0) {

    recvPos = 0;

    while ((waitTime = millis() - startTs) <= timeout) {
      int currentByte;
      while (app_uart_get(&currentByte) != NRF_SUCCESS);

      if (currentByte < 0) {
        continue;
      }

      switch (recvPos) {
      case 0: // first byte == AA
        if (currentByte != (PH & 0xFF)) {
          continue;
        }

        break;

      case 1: // second byte == 55
        CheckSumCal = PH;

        if (currentByte != (PH >> 8)) {
          recvPos = 0;
          continue;
        }

        break;

      case 2: // package type
        SampleNumlAndCTCal = currentByte;

        if (((currentByte&0x01) != CT_Normal) && ((currentByte & 0x01) != CT_RingStart)) {
          recvPos = 0;
          continue;
        }

        break;

      case 3: // sample quantity
        SampleNumlAndCTCal += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        package_Sample_Num = currentByte;
        break;

      case 4: // start angle
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          FirstSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }

        break;

      case 5: // start angle
        FirstSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        CheckSumCal ^= FirstSampleAngle;
        FirstSampleAngle = FirstSampleAngle >> 1;
        break;

      case 6: //end angle
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          LastSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }

        break;

      case 7: // end angle
        LastSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        LastSampleAngleCal = LastSampleAngle;
        LastSampleAngle = LastSampleAngle >> 1;

        if (package_Sample_Num == 1) {
          IntervalSampleAngle = 0;
        } else {
          if (LastSampleAngle < FirstSampleAngle) {
            if ((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)) {
              IntervalSampleAngle = ((float)(23040 + LastSampleAngle - FirstSampleAngle)) /
                                    (package_Sample_Num - 1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            } else {
              IntervalSampleAngle = IntervalSampleAngle_LastPackage;
            }
          } else {
            IntervalSampleAngle = ((float)(LastSampleAngle - FirstSampleAngle)) / (package_Sample_Num - 1);
            IntervalSampleAngle_LastPackage = IntervalSampleAngle;
          }
        }

        break;

      case 8: //checksum
        CheckSum = currentByte;
        break;

      case 9: //checksum
        CheckSum += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        break;
      }

      packageBuffer[recvPos++] = currentByte;

      if (recvPos  == PackagePaidBytes) {
        package_recvPos = recvPos;
        break;

      }
    }

    if (PackagePaidBytes == recvPos) {
      startTs = millis();
      recvPos = 0;
      int package_sample_sum = package_Sample_Num << 1;

      while ((waitTime = millis() - startTs) <= timeout) {
       int currentByte;
       while (app_uart_get(&currentByte) != NRF_SUCCESS);

        if (currentByte < 0) {
          continue;
        }

        if ((recvPos & 1) == 1) {
          Valu8Tou16 += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          CheckSumCal ^= Valu8Tou16;
        } else {
          Valu8Tou16 = currentByte;
        }

        packageBuffer[package_recvPos + recvPos] = currentByte;
        recvPos++;

        if (package_sample_sum == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_sample_sum != recvPos) {
        return RESULT_FAIL;
      }
    } else {
      return RESULT_FAIL;
    }

    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if (CheckSumCal != CheckSum) {
      CheckSumResult = false;
    } else {
      CheckSumResult = true;
    }

  }

  uint8_t package_CT;
  package_CT = package.package_CT;

  if ((package_CT&0x01) == CT_Normal) {
    node.sync_quality = Node_Default_Quality + Node_NotSync;
  } else {
    node.sync_quality = Node_Default_Quality + Node_Sync;
  }

  if (CheckSumResult == true) {
    node.distance_q2 = package.packageSampleDistance[package_Sample_Index];

    if (node.distance_q2 / 4 != 0) {
      AngleCorrectForDistance = (int32_t)((atan(((21.8 * (155.3 - (node.distance_q2 * 0.25f))) /
                                           155.3) / (node.distance_q2 * 0.25f))) * 3666.93);
    } else {
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = IntervalSampleAngle * package_Sample_Index;

    if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0) {
      node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance +
                                            23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
        node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance -
                                              23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance)) <<
                                  LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    node.sync_quality = Node_Default_Quality + Node_NotSync;
    node.angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    node.distance_q2 = 0;
    package_Sample_Index = 0;
    return RESULT_FAIL;
  }

  point.distance = node.distance_q2 * 0.25f;
  point.angle = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
  point.quality = (node.sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
  point.startBit = (node.sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT);

  package_Sample_Index++;
  nowPackageNum = package.nowPackageNum;

  if (package_Sample_Index >= nowPackageNum) {
    package_Sample_Index = 0;
  }

  return RESULT_OK;
}


// wait response header
result_t waitResponseHeader(lidar_ans_header *header, uint32_t timeout) {
  int  recvPos = 0;
  uint32_t startTs = millis();
  uint8_t  *headerBuffer = (uint8_t *)(header);
  uint32_t waitTime;

  while ((waitTime = millis() - startTs) <= timeout) {
    int currentbyte;
    while (app_uart_get(&currentbyte) != NRF_SUCCESS);

    if (currentbyte < 0) {
      continue;
    }

    switch (recvPos) {
    case 0:
      if (currentbyte != LIDAR_ANS_SYNC_BYTE1) {
        continue;
      }

      break;

    case 1:
      if (currentbyte != LIDAR_ANS_SYNC_BYTE2) {
        recvPos = 0;
        continue;
      }

      break;
    }

    headerBuffer[recvPos++] = currentbyte;

    if (recvPos == sizeof(lidar_ans_header)) {
      return RESULT_OK;
    }
  }

  return RESULT_TIMEOUT;
}

scanPoint getCurrentScanPoint(){
	return point;
}

