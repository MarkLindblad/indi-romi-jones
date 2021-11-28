/* modified YDLidar arduino library to work on nrf52 dk
*/
#include "YDLidar.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_error.h"
#include <nrf_serial.h>


extern const nrf_serial_t * lidar_serial;
result_t waitResponseHeader(lidar_ans_header *header, uint32_t timeout);

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



scanPoint point = {0,0,0,NULL};


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

// parses one package
result_t haveData(uint32_t timeout) {
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

    while (compareMillis(startTs, millis()) <= timeout) {
      int currentByte = 0;

      while (nrf_serial_read(lidar_serial, &currentByte, 1, NULL, 20)!= NRF_SUCCESS);

      if (currentByte < 0) {
        continue;
      }
      // printf("%x", currentByte);

      switch (recvPos) {
        
      case 0: // first byte == A
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

      while (compareMillis(startTs, millis()) <= timeout) {
       int currentByte;
       while (nrf_serial_read(lidar_serial, &currentByte, 1, NULL, 20) != NRF_SUCCESS);

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
    int currentByte;
    while (nrf_serial_read(lidar_serial, &currentByte, 1, NULL, 20) != NRF_SUCCESS);

    if (currentByte < 0) {
      continue;
    }

    switch (recvPos) {
    case 0:
      if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
        continue;
      }

      break;

    case 1:
      if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
        recvPos = 0;
        continue;
      }

      break;
    }

    headerBuffer[recvPos++] = currentByte;

    if (recvPos == sizeof(lidar_ans_header)) {
      return RESULT_OK;
    }
  }

  return RESULT_TIMEOUT;
}

scanPoint getCurrentScanPoint(){
	return point;
}





/*
0816d10816110815110814510813910812d10812110811910810d1081051081f91071c11071801071b21061cd1061
aa1551001281d71851171951df16b1c91061bd1061c91
061cd1061d91061e91061051071e510612e10815a10c1aa1071a91071a51071a51071a11071a11071a11071a11071a11071a51071a51071a91071a51071a91071ad1071ad1
071b11071b51071b91071bd1071d110719e10419c1041ae1011b51011a21041b11041b91041c11041c51041
aa1551001281751951b71a41e914d1d91041c61011c41011c810
11c81011d01011f61041f41041e0104100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100
1001001001001001001001001001001001001921061b01061c81061dc10610010011e1071401071aa15510011211d1a51c31ab1d21451601071811071a91071d010710010010
010010010010610d13510d17510d1b910d1a910d14910e19110e1e110e13510f18d10f1e910f1aa1551011011291ac1291ac1921441391101aa15510012818f1ac1d910718e1
e219a10d1c810d1a211111511219d1121291131c511317611413a11511211610a11710611812211915611a1a211b11211d19611e12a1201da1211b21231ce12516a12817212
b12212f1de13618a13b1c113b1c513b1b913b1ad13b18513b17913b16513b15513b15113b13d13b14913b14d13b16913b1d913b1aa15510012813f10817711714e16315610413
d10413110412d10412910412510412510412510412510412910412d1041311041391041411041491041551041611041691041791041851041991041ad1041c11041d91041f1104
10510512110513910515510513210512510511d1051b610416a10415d10415110414d10414d1041491041511041aa1551001281dd11711d12717614d1551041611041651041711
0417510417910417510417510416910416110415910415910415510415910415910415910415d10416110416110415d10415d10415910415110414d10414510413d10413d1041391
0413510413910413910414110414110414910415510416110416d10417d1041811041891041aa1551001281831271cf13612015a19110419510419d10419d1041a51041ad1041b
51041c11041c51041d11041d51041e11041f510411e10514210515510516110516910516910516d10517d1051b61051cd1051dd1051f11051fd1051ed1051e51051e910511a1061
001001001001001001001001001001f61331051341251341411341d51331aa15510012812b13718114610613e1721321761341991341b91341e91341091351ad13515610214d102
14510214110214110213d10213d10213d10213910213510213510213510213110212d10212910212910212510212110212110211d10211910211910211510211110210d102111102
*/
