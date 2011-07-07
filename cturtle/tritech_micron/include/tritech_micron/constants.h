// ######################################################################
//
//      TritechMicron - A protocol parser for Tritech Micron sonars.
//      Copyright (C) 2011  Randolph Voorhies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ######################################################################

#ifndef TRITECHMICRON_CONSTANTS_H
#define TRITECHMICRON_CONSTANTS_H

namespace tritech
{
  enum MessageType
  {
    mtNull                = 0,
    mtVersionData         = 1,
    mtHeadData            = 2,
    mtSpectData           = 3,
    mtAlive               = 4,
    mtPrgAck              = 5,
    mtBBUserData          = 6,
    mtTestData            = 7,
    mtAuxData             = 8, 
    mtAdcData             = 9, 
    mtAdcReq              = 10,
    mtLanStatus           = 13,
    mtSetTime             = 14,
    mtTimeout             = 15,
    mtReBoot              = 16,
    mtPerformanceData     = 17, //What, no 18?
    mtHeadCommand         = 19,
    mtEraseSector         = 20,
    mtProgBlock           = 21,
    mtCopyBootBlk         = 22,
    mtSendVersion         = 23,
    mtSendBBuser          = 24,
    mtSendData            = 25,
    mtSendPerformanceData = 26,
    mtDopplerData         = 27,
    mtDopplerParams       = 28,
    mtAttitudeData        = 29,
    mtAttitudeParams      = 30,
    mtBathyParams         = 31,
    mtBathyData           = 32,
    mtWspData             = 33,
    mtWspParams           = 34,
    mtStreamData          = 35,
    mtGenericData         = 36,
    mtGUID_Data           = 37,
    mtIPAQCntrlParam      = 38,
    mtIPAQCntrlData       = 39,
    mtFpgaTest            = 40,
    mtFpgaErase           = 41,
    mtFpgaProgram         = 42,
    mtBathyInfo           = 43,
    mtBathyProfile        = 44,
    mtSendBathyPrfReq     = 45,
    mtSendBathyProfile    = 46,
    mtSendFpgaFlashSt     = 47,
    mtFpgaTestData        = 48,
    mtFpgaFlashStData     = 49,
    mtSendTrnspdrStat     = 50,
    mtSendTrnspdrData     = 51,
    mtAMNavData           = 52,
    mtAMNavParams         = 53,
    mtTrnspdrTxCntrl      = 54,
    mtTrnspdrConfig       = 55,
    mtSendFpgaVersion     = 56,
    mtFpgaVersionData     = 57,
    mtScanHeader          = 58,
    mtScanData            = 59,
    mtGlobal              = 60,
    mtFpgaDoCalibrate     = 61,
    mtSendFpgaCalData     = 62,
    mtFpgaCalData         = 63,
    mZeroFpgaCal          = 64,
    mtSWParams            = 65,
    mtStopAlives          = 66,
    mtResponderPing       = 67,
    mtVideoCntrlCmd       = 68,
    mtVideoCntrlData      = 69,
    mtResetToDefaults     = 70,
    mtChangeVerData       = 71,
    mtFpgaProgUsrCde      = 72
  };

}

#endif // TRITECHMICRON_CONSTANTS_H

