// **********************************************************************************
// Registers used in driver definition for HopeRF RFM69HCW
// **********************************************************************************

#define RFM_CLK_OUTPUT 0

// RFM69HCW Crystal Oscillator Frq
#define RFM69_OSC_FRQ			(32000000L) 

// **************************************************
// Internal registers addresses
// **************************************************
#define REG_FIFO			                0x00
#define REG_OPMODE			  	        0x01
#define REG_DATAMODUL	  		                0x02
#define REG_BITRATE				        0x03
#define REG_BITRATEMSB			                0x03
#define REG_BITRATELSB			                0x04
#define REG_FDEV			  	        0x05
#define REG_FDEVMSB			  	        0x05
#define REG_FDEVLSB		  		        0x06
#define REG_FRF		  			        0x07
#define REG_FRFMSB	  			        0x07
#define REG_FRFMID			  	        0x08
#define REG_FRFLSB		  		        0x09
#define REG_OSC1		  	  	        0x0A
#define REG_AFCCTRL 		  	                0x0B

#define REG_LISTEN1			  	        0x0D
#define REG_LISTEN2			  	        0x0E
#define REG_LISTEN3			  	        0x0F
#define REG_VERSION			  	        0x10
#define REG_PALEVEL			  	        0x11
#define REG_PARAMP			  	        0x12
#define REG_OCP				                0x13

#define REG_LNA			  	  	        0x18
#define REG_RXBW		    	                0x19
#define REG_AFCBW		    	                0x1A
#define REG_OOKPEAK	  			        0x1B
#define REG_OOKAVG  			                0x1C
#define REG_OOKFIX	  			        0x1D
#define REG_AFCFEI		  		        0x1E
#define REG_AFCMSB			  	        0x1F
#define REG_AFCLSB			  	        0x20
#define REG_FEIMSB			  	        0x21
#define REG_FEILSB			  	        0x22
#define REG_RSSICONFIG			                0x23
#define REG_RSSIVALUE		  	                0x24
#define REG_DIOMAPPING1			                0x25
#define REG_DIOMAPPING2			                0x26
#define REG_IRQFLAGS1		  	                0x27
#define REG_IRQFLAGS2		  	                0x28
#define REG_RSSITHRESH			                0x29
#define REG_RXTIMEOUT1			                0x2A
#define REG_RXTIMEOUT2			                0x2B
#define REG_PREAMBLE			                0x2C
#define REG_PREAMBLEMSB			                0x2C
#define REG_PREAMBLELSB			                0x2D
#define REG_SYNCCONFIG			                0x2E
#define REG_SYNCVALUE1			                0x2F
#define REG_SYNCVALUE2			                0x30
#define REG_SYNCVALUE3			                0x31
#define REG_SYNCVALUE4			                0x32
#define REG_SYNCVALUE5			                0x33
#define REG_SYNCVALUE6			                0x34
#define REG_SYNCVALUE7			                0x35
#define REG_SYNCVALUE8			                0x36
#define REG_PACKETCONFIG1		                0x37
#define REG_PAYLOADLENGTH		                0x38
#define REG_NODEADRS		                  	0x39
#define REG_BROADCASTADRS	                	0x3A
#define REG_AUTOMODES		  	                0x3B
#define REG_FIFOTHRESH			                0x3C
#define REG_PACKETCONFIG2		                0x3D
#define REG_AESKEY1			  	        0x3E
#define REG_AESKEY2 			                0x3F
#define REG_AESKEY3	  			        0x40
#define REG_AESKEY4		  		        0x41
#define REG_AESKEY5			  	        0x42
#define REG_AESKEY6 			                0x43
#define REG_AESKEY7	  			        0x44
#define REG_AESKEY8		  		        0x45
#define REG_AESKEY9			  	        0x46
#define REG_AESKEY10  			                0x47
#define REG_AESKEY11	  		                0x48
#define REG_AESKEY12		                  	0x49
#define REG_AESKEY13  			                0x4A
#define REG_AESKEY14	  		                0x4B
#define REG_AESKEY15		  	                0x4C
#define REG_AESKEY16		  	                0x4D
#define REG_TEMP1			                0x4E
#define REG_TEMP2			                0x4F

#define REG_TESTLNA       		                0x58
#define REG_TESTPA1       		                0x5A
#define REG_TESTPA2       		                0x5C
#define REG_TESTDAGC      		                0x6F
#define REG_TESTAFC                                     0x71

//******************************************************
// RF69/SX1231 bit control definition
//******************************************************
// RegOpMode
#define RF_OPMODE_SEQUENCER_OFF				0x80
#define RF_OPMODE_SEQUENCER_ON				0x00  // Default

#define RF_OPMODE_LISTEN_ON				0x40
#define RF_OPMODE_LISTEN_OFF				0x00  // Default

#define RF_OPMODE_LISTENABORT				0x20

#define RF_OPMODE_SLEEP					0x00
#define RF_OPMODE_STANDBY				0x04  // Default
#define RF_OPMODE_SYNTHESIZER				0x08
#define RF_OPMODE_TRANSMITTER				0x0C
#define RF_OPMODE_RECEIVER				0x10

// RegDataModul
#define RF_DATAMODUL_DATAMODE_PACKET			0x00  // Default
#define RF_DATAMODUL_DATAMODE_CONTINUOUS		0x40
#define RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC	0x60

#define RF_DATAMODUL_MODULATIONTYPE_FSK			0x00  // Default
#define RF_DATAMODUL_MODULATIONTYPE_OOK			0x08

#define RF_DATAMODUL_MODULATIONSHAPING_00		0x00  // Default
#define RF_DATAMODUL_MODULATIONSHAPING_01		0x01
#define RF_DATAMODUL_MODULATIONSHAPING_10		0x02
#define RF_DATAMODUL_MODULATIONSHAPING_11		0x03

// Compute Datarate from given baudrate ( see Manual pg 19 )
#define RFM69_DATARATE(baud)				( RFM69_OSC_FRQ / baud )


// RegFdev - frequency deviation (Hz)
#define RF_FDEV_2000					0x0021
#define RF_FDEV_5000					0x0052  // Default
#define RF_FDEV_7500					0x007B
#define RF_FDEV_10000					0x00A4
#define RF_FDEV_15000					0x00F6
#define RF_FDEV_20000					0x0148
#define RF_FDEV_25000					0x019A
#define RF_FDEV_30000					0x01EC
#define RF_FDEV_35000					0x023D
#define RF_FDEV_40000					0x028F
#define RF_FDEV_45000					0x02E1
#define RF_FDEV_50000					0x0333
#define RF_FDEV_55000					0x0385
#define RF_FDEV_60000					0x03D7
#define RF_FDEV_65000					0x0429
#define RF_FDEV_70000					0x047B
#define RF_FDEV_75000					0x04CD
#define RF_FDEV_80000					0x051F
#define RF_FDEV_85000					0x0571
#define RF_FDEV_90000					0x05C3
#define RF_FDEV_95000					0x0614
#define RF_FDEV_100000					0x0666
#define RF_FDEV_110000					0x070A
#define RF_FDEV_120000					0x07AE
#define RF_FDEV_130000					0x0852
#define RF_FDEV_140000					0x08F6
#define RF_FDEV_150000					0x099A
#define RF_FDEV_160000					0x0A3D
#define RF_FDEV_170000					0x0AE1
#define RF_FDEV_180000					0x0B85
#define RF_FDEV_190000					0x0C29
#define RF_FDEV_200000					0x0CCD
#define RF_FDEV_210000					0x0D71
#define RF_FDEV_220000					0x0E14
#define RF_FDEV_230000					0x0EB8
#define RF_FDEV_240000					0x0F5C
#define RF_FDEV_250000					0x1000
#define RF_FDEV_260000					0x10A4
#define RF_FDEV_270000					0x1148
#define RF_FDEV_280000					0x11EC
#define RF_FDEV_290000					0x128F
#define RF_FDEV_300000					0x1333

// RegFrf (MHz) - carrier frequency
#define RFM69_FRQ(v) 					(uint32_t)((v)*1000000.0/RFM69_OSC_FRQ * (1L<<19)+0.5) 
#define RFM69_FRQ_MSB(v) 				((RFM69_FRQ(v) >> 16) & 0xFF )
#define RFM69_FRQ_MID(v) 				((RFM69_FRQ(v) >> 8)  & 0xFF )
#define RFM69_FRQ_LSB(v) 				((RFM69_FRQ(v)     )  & 0xFF )


// Some predefined values, not needed
#define RF_FRF_314					0x4E8000
#define RF_FRF_315					0x4EC000
#define RF_FRF_316					0x4F0000
// 433mhz band
#define RF_FRF_433					0x6C4000
#define RF_FRF_434					0x6C8000
#define RF_FRF_435					0x6CC000
// 868Mhz band
#define RF_FRF_863					0xD7C000
#define RF_FRF_864					0xD80000
#define RF_FRF_865					0xD84000
#define RF_FRF_866					0xD88000
#define RF_FRF_867					0xD8C000
#define RF_FRF_868					0xD90000
#define RF_FRF_869					0xD94000
#define RF_FRF_870					0xD98000
// 915Mhz band
#define RF_FRF_902					0xE18000
#define RF_FRF_903					0xE1C000
#define RF_FRF_904					0xE20000
#define RF_FRF_905					0xE24000
#define RF_FRF_906					0xE28000
#define RF_FRF_907					0xE2C000
#define RF_FRF_908					0xE30000
#define RF_FRF_909					0xE34000
#define RF_FRF_910					0xE38000
#define RF_FRF_911					0xE3C000
#define RF_FRF_912					0xE40000
#define RF_FRF_913					0xE44000
#define RF_FRF_914					0xE48000
#define RF_FRF_915					0xE4C000  // Default
#define RF_FRF_916					0xE50000
#define RF_FRF_917					0xE54000
#define RF_FRF_918					0xE58000
#define RF_FRF_919					0xE5C000
#define RF_FRF_920					0xE60000
#define RF_FRF_921					0xE64000
#define RF_FRF_922					0xE68000
#define RF_FRF_923					0xE6C000
#define RF_FRF_924					0xE70000
#define RF_FRF_925					0xE74000
#define RF_FRF_926					0xE78000
#define RF_FRF_927					0xE7C000
#define RF_FRF_928					0xE80000


// RegOsc1
#define RF_OSC1_RCCAL_START				0x80
#define RF_OSC1_RCCAL_DONE				0x40

// RegListen1
#define RF_LISTEN1_RESOL_64				0x50
#define RF_LISTEN1_RESOL_4100				0xA0  // Default
#define RF_LISTEN1_RESOL_262000				0xF0

#define RF_LISTEN1_CRITERIA_RSSI			0x00  // Default
#define RF_LISTEN1_CRITERIA_RSSIANDSYNC	  		0x08

#define RF_LISTEN1_END_00				0x00
#define RF_LISTEN1_END_01				0x02  // Default
#define RF_LISTEN1_END_10				0x04


// RegPaLevel
#define RF_PALEVEL_PA0_ON		  		0x80  // Default
#define RF_PALEVEL_PA1_ON				0x40
#define RF_PALEVEL_PA2_ON				0x20
#define RF_PALEVEL_OFF					0x00  // Default

#define RF_PALEVEL_OUTPUTPOWER_MIN			0x00
#define RF_PALEVEL_OUTPUTPOWER_MAX			0x1F  // Default
#define RF_OUTPUTPOWER(lvl) 				(lvl > RF_PALEVEL_OUTPUTPOWER_MAX ? RF_PALEVEL_OUTPUTPOWER_MAX : lvl )

// RegPaRamp: Signal Rise and fall time in ms in FSK mode
#define RF_PARAMP_3400					0x00
#define RF_PARAMP_2000					0x01
#define RF_PARAMP_1000					0x02
#define RF_PARAMP_500					0x03
#define RF_PARAMP_250					0x04
#define RF_PARAMP_125					0x05
#define RF_PARAMP_100					0x06
#define RF_PARAMP_62					0x07
#define RF_PARAMP_50					0x08
#define RF_PARAMP_40					0x09  // Default
#define RF_PARAMP_31					0x0A
#define RF_PARAMP_25					0x0B
#define RF_PARAMP_20					0x0C
#define RF_PARAMP_15					0x0D
#define RF_PARAMP_12					0x0E
#define RF_PARAMP_10					0x0F


// RegOcp
#define RF_OCP_OFF					0x0F
#define RF_OCP_ON					0x1A  // Default

#define RF_OCP_TRIM_45					0x00
#define RF_OCP_TRIM_50					0x01
#define RF_OCP_TRIM_55					0x02
#define RF_OCP_TRIM_60					0x03
#define RF_OCP_TRIM_65					0x04
#define RF_OCP_TRIM_70					0x05
#define RF_OCP_TRIM_75					0x06
#define RF_OCP_TRIM_80					0x07
#define RF_OCP_TRIM_85					0x08
#define RF_OCP_TRIM_90					0x09
#define RF_OCP_TRIM_95					0x0A
#define RF_OCP_TRIM_100					0x0B  // Default
#define RF_OCP_TRIM_105					0x0C
#define RF_OCP_TRIM_110					0x0D
#define RF_OCP_TRIM_115					0x0E
#define RF_OCP_TRIM_120					0x0F



// RegLna
#define RF_LNA_ZIN_50					0x00
#define RF_LNA_ZIN_200					0x80  // Default

#define RF_LNA_LOWPOWER_OFF				0x00  // Default
#define RF_LNA_LOWPOWER_ON				0x40

#define RF_LNA_CURRENTGAIN				0x08

#define RF_LNA_GAINSELECT_AUTO				0x00  // Default
#define RF_LNA_GAINSELECT_MAX				0x01
#define RF_LNA_GAINSELECT_MAXMINUS6			0x02
#define RF_LNA_GAINSELECT_MAXMINUS12			0x03
#define RF_LNA_GAINSELECT_MAXMINUS24			0x04
#define RF_LNA_GAINSELECT_MAXMINUS36			0x05
#define RF_LNA_GAINSELECT_MAXMINUS48			0x06


// RegRxBw
#define RF_RXBW_DCCFREQ_000				0x00
#define RF_RXBW_DCCFREQ_001				0x20
#define RF_RXBW_DCCFREQ_010				0x40  // Default
#define RF_RXBW_DCCFREQ_011				0x60
#define RF_RXBW_DCCFREQ_100				0x80
#define RF_RXBW_DCCFREQ_101				0xA0
#define RF_RXBW_DCCFREQ_110				0xC0
#define RF_RXBW_DCCFREQ_111				0xE0

#define RF_RXBW_FSK_2_6					0b10111
#define RF_RXBW_FSK_3_1					0b01111
#define RF_RXBW_FSK_3_9					0b00111
#define RF_RXBW_FSK_5_2					0b10110
#define RF_RXBW_FSK_6_3					0b01110
#define RF_RXBW_FSK_7_8					0b00110
#define RF_RXBW_FSK_10_4				0b10101	// Default
#define RF_RXBW_FSK_12_5				0b01101
#define RF_RXBW_FSK_15_6				0b00101
#define RF_RXBW_FSK_20_8				0b10100
#define RF_RXBW_FSK_25_0				0b01100
#define RF_RXBW_FSK_31_3				0b00100
#define RF_RXBW_FSK_41_7				0b10011
#define RF_RXBW_FSK_50_0				0b01011
#define RF_RXBW_FSK_62_5				0b00011
#define RF_RXBW_FSK_83_3				0b10010
#define RF_RXBW_FSK_100					0b01010
#define RF_RXBW_FSK_125					0b00010
#define RF_RXBW_FSK_167					0b10001
#define RF_RXBW_FSK_200					0b01001
#define RF_RXBW_FSK_250					0b00001
#define RF_RXBW_FSK_333					0b10000
#define RF_RXBW_FSK_400					0b01000
#define RF_RXBW_FSK_500					0b00000

#define RF_GET_RXBW(baud)                               ( ( (baud)<8000 ? RF_RXBW_FSK_62_5 : ( (baud)<30000 ? RF_RXBW_FSK_125 : RF_RXBW_FSK_200 ) ) | RF_RXBW_DCCFREQ_010 )


#define RF_RXBW_OOK_1_3					0b10111
#define RF_RXBW_OOK_1_6					0b01111
#define RF_RXBW_OOK_2_0					0b00111
#define RF_RXBW_OOK_2_6					0b10110
#define RF_RXBW_OOK_3_1					0b01110
#define RF_RXBW_OOK_3_9					0b00110
#define RF_RXBW_OOK_5_2					0b10101 // Default
#define RF_RXBW_OOK_6_3					0b01101
#define RF_RXBW_OOK_7_8					0b00101
#define RF_RXBW_OOK_10_4				0b10100
#define RF_RXBW_OOK_12_5				0b01100
#define RF_RXBW_OOK_15_6				0b00100
#define RF_RXBW_OOK_20_8				0b10011
#define RF_RXBW_OOK_25_0				0b01011
#define RF_RXBW_OOK_31_3				0b00011
#define RF_RXBW_OOK_41_7				0b10010
#define RF_RXBW_OOK_50					0b01010
#define RF_RXBW_OOK_62_5				0b00010
#define RF_RXBW_OOK_83_3				0b10001
#define RF_RXBW_OOK_100					0b01001
#define RF_RXBW_OOK_125					0b00001
#define RF_RXBW_OOK_167					0b10000
#define RF_RXBW_OOK_200					0b01000
#define RF_RXBW_OOK_250					0b00000


// RegAfcBw
#define RF_AFCBW_DCCFREQAFC_000				0x00
#define RF_AFCBW_DCCFREQAFC_001				0x20
#define RF_AFCBW_DCCFREQAFC_010				0x40
#define RF_AFCBW_DCCFREQAFC_011				0x60
#define RF_AFCBW_DCCFREQAFC_100				0x80  // Default
#define RF_AFCBW_DCCFREQAFC_101				0xA0
#define RF_AFCBW_DCCFREQAFC_110				0xC0
#define RF_AFCBW_DCCFREQAFC_111				0xE0

#define RF_AFCBW_MANTAFC_16				0x00
#define RF_AFCBW_MANTAFC_20				0x08  // Default
#define RF_AFCBW_MANTAFC_24				0x10

#define RF_AFCBW_EXPAFC_0				0x00
#define RF_AFCBW_EXPAFC_1	  			0x01
#define RF_AFCBW_EXPAFC_2		  		0x02
#define RF_AFCBW_EXPAFC_3			  	0x03  // Default
#define RF_AFCBW_EXPAFC_4				0x04
#define RF_AFCBW_EXPAFC_5                               0x05
#define RF_AFCBW_EXPAFC_6				0x06
#define RF_AFCBW_EXPAFC_7                               0x07


// RegOokPeak
#define RF_OOKPEAK_THRESHTYPE_FIXED			0x00
#define RF_OOKPEAK_THRESHTYPE_PEAK			0x40  // Default
#define RF_OOKPEAK_THRESHTYPE_AVERAGE			0x80

#define RF_OOKPEAK_PEAKTHRESHSTEP_000			0x00  // Default
#define RF_OOKPEAK_PEAKTHRESHSTEP_001			0x08
#define RF_OOKPEAK_PEAKTHRESHSTEP_010			0x10
#define RF_OOKPEAK_PEAKTHRESHSTEP_011			0x18
#define RF_OOKPEAK_PEAKTHRESHSTEP_100			0x20
#define RF_OOKPEAK_PEAKTHRESHSTEP_101			0x28
#define RF_OOKPEAK_PEAKTHRESHSTEP_110			0x30
#define RF_OOKPEAK_PEAKTHRESHSTEP_111			0x38

#define RF_OOKPEAK_PEAKTHRESHDEC_000			0x00  // Default
#define RF_OOKPEAK_PEAKTHRESHDEC_001			0x01
#define RF_OOKPEAK_PEAKTHRESHDEC_010			0x02
#define RF_OOKPEAK_PEAKTHRESHDEC_011			0x03
#define RF_OOKPEAK_PEAKTHRESHDEC_100			0x04
#define RF_OOKPEAK_PEAKTHRESHDEC_101			0x05
#define RF_OOKPEAK_PEAKTHRESHDEC_110			0x06
#define RF_OOKPEAK_PEAKTHRESHDEC_111			0x07


// RegOokAvg
#define RF_OOKAVG_AVERAGETHRESHFILT_00			0x00
#define RF_OOKAVG_AVERAGETHRESHFILT_01			0x40
#define RF_OOKAVG_AVERAGETHRESHFILT_10			0x80  // Default
#define RF_OOKAVG_AVERAGETHRESHFILT_11			0xC0


// RegOokFix
#define RF_OOKFIX_FIXEDTHRESH_VALUE			0x06  // Default


// RegAfcFei
#define RF_AFCFEI_FEI_DONE				0x40
#define RF_AFCFEI_FEI_START				0x20
#define RF_AFCFEI_AFC_DONE				0x10
#define RF_AFCFEI_AFCAUTOCLEAR_ON			0x08
#define RF_AFCFEI_AFCAUTOCLEAR_OFF			0x00  // Default

#define RF_AFCFEI_AFCAUTO_ON				0x04
#define RF_AFCFEI_AFCAUTO_OFF				0x00  // Default

#define RF_AFCFEI_AFC_CLEAR				0x02
#define RF_AFCFEI_AFC_START                             0x01

// RegRssiConfig
#define RF_RSSI_FASTRX_ON				0x08
#define RF_RSSI_FASTRX_OFF				0x00  // Default
#define RF_RSSI_DONE					0x02
#define RF_RSSI_START					0x01


// RegDioMapping1
#define RF_DIOMAPPING1_DIO0_00                          0x00  // Default
#define RF_DIOMAPPING1_DIO0_01                          0x40
#define RF_DIOMAPPING1_DIO0_10			  	0x80
#define RF_DIOMAPPING1_DIO0_11				0xC0

#define RF_DIOMAPPING1_DIO1_00   			0x00  // Default
#define RF_DIOMAPPING1_DIO1_01		  		0x10
#define RF_DIOMAPPING1_DIO1_10			  	0x20
#define RF_DIOMAPPING1_DIO1_11				0x30

#define RF_DIOMAPPING1_DIO2_00	  			0x00  // Default
#define RF_DIOMAPPING1_DIO2_01		  		0x04
#define RF_DIOMAPPING1_DIO2_10			  	0x08
#define RF_DIOMAPPING1_DIO2_11				0x0C

#define RF_DIOMAPPING1_DIO3_00	  			0x00  // Default
#define RF_DIOMAPPING1_DIO3_01		  		0x01
#define RF_DIOMAPPING1_DIO3_10			  	0x02
#define RF_DIOMAPPING1_DIO3_11				0x03


// RegDioMapping2
#define RF_DIOMAPPING2_DIO4_00	  			0x00  // Default
#define RF_DIOMAPPING2_DIO4_01                          0x40
#define RF_DIOMAPPING2_DIO4_10                          0x80
#define RF_DIOMAPPING2_DIO4_11				0xC0

#define RF_DIOMAPPING2_DIO5_00	  			0x00  // Default
#define RF_DIOMAPPING2_DIO5_01		  		0x10
#define RF_DIOMAPPING2_DIO5_10			  	0x20
#define RF_DIOMAPPING2_DIO5_11				0x30

#define RF_DIOMAPPING2_CLKOUT_32	  		0x00
#define RF_DIOMAPPING2_CLKOUT_16		  	0x01
#define RF_DIOMAPPING2_CLKOUT_8				0x02
#define RF_DIOMAPPING2_CLKOUT_4				0x03
#define RF_DIOMAPPING2_CLKOUT_2		  		0x04
#define RF_DIOMAPPING2_CLKOUT_1			        0x05
#define RF_DIOMAPPING2_CLKOUT_RC			0x06
#define RF_DIOMAPPING2_CLKOUT_OFF			0x07  // Default


// RegIrqFlags1
#define RF_IRQFLAGS1_MODEREADY				0x80
#define RF_IRQFLAGS1_RXREADY				0x40
#define RF_IRQFLAGS1_TXREADY				0x20
#define RF_IRQFLAGS1_PLLLOCK				0x10
#define RF_IRQFLAGS1_RSSI				0x08
#define RF_IRQFLAGS1_TIMEOUT				0x04
#define RF_IRQFLAGS1_AUTOMODE				0x02
#define RF_IRQFLAGS1_SYNCADDRESSMATCH			0x01

// RegIrqFlags2
#define RF_IRQFLAGS2_FIFOFULL				0x80
#define RF_IRQFLAGS2_FIFONOTEMPTY			0x40
#define RF_IRQFLAGS2_FIFOLEVEL				0x20
#define RF_IRQFLAGS2_FIFOOVERRUN			0x10
#define RF_IRQFLAGS2_PACKETSENT				0x08
#define RF_IRQFLAGS2_PAYLOADREADY			0x04
#define RF_IRQFLAGS2_CRCOK				0x02


// RegRssiThresh
#define RF_RSSITHRESH_VALUE				0xE4  // Default

// RegRxTimeout1
#define RF_RXTIMEOUT1_RXSTART_VALUE			0x00  // Default

// RegRxTimeout2
#define RF_RXTIMEOUT2_RSSITHRESH_VALUE			0x00  // Default

// Preamble size
#define RFM69_PREAMBLE_SIZE_MSB(v) 			( (v >> 8 ) & 0xFF )
#define RFM69_PREAMBLE_SIZE_LSB(v)			( v & 0xFF )


// RegSyncConfig
#define RF_SYNC_ON					0x80  // Default
#define RF_SYNC_OFF					0x00

#define RF_SYNC_FIFOFILL_AUTO				0x00  // Default -- when sync interrupt occurs
#define RF_SYNC_FIFOFILL_MANUAL				0x40

// Number of sync bytes
#define RF_SYNC_SIZE_1					0x00
#define RF_SYNC_SIZE_2					0x08
#define RF_SYNC_SIZE_3					0x10
#define RF_SYNC_SIZE_4					0x18  // Default
#define RF_SYNC_SIZE_5					0x20
#define RF_SYNC_SIZE_6					0x28
#define RF_SYNC_SIZE_7					0x30
#define RF_SYNC_SIZE_8					0x38

// Number of tolerated bit errors in sync bytes
#define RF_SYNC_TOL_0					0x00  // Default
#define RF_SYNC_TOL_1					0x01
#define RF_SYNC_TOL_2					0x02
#define RF_SYNC_TOL_3					0x03
#define RF_SYNC_TOL_4					0x04
#define RF_SYNC_TOL_5					0x05
#define RF_SYNC_TOL_6					0x06
#define RF_SYNC_TOL_7					0x07


// RegPacketConfig1
#define RF_PACKET_FORMAT_FIXED				0x00  // Default
#define RF_PACKET_FORMAT_VARIABLE			0x80

#define RF_PACKET_DCFREE_OFF				0x00  // Default
#define RF_PACKET_DCFREE_MANCHESTER			0x20
#define RF_PACKET_DCFREE_WHITENING			0x40

#define RF_PACKET_CRC_ON				0x10  // Default
#define RF_PACKET_CRC_OFF				0x00

#define RF_PACKET_CRCAUTOCLEAR_ON			0x00  // Default
#define RF_PACKET_CRCAUTOCLEAR_OFF			0x08

#define RF_PACKET_ADRSFILTERING_OFF                     0x00  // Default
#define RF_PACKET_ADRSFILTERING_NODE			0x02
#define RF_PACKET_ADRSFILTERING_NODEBROADCAST           0x04


// RegAutoModes
#define RF_AUTOMODES_ENTER_OFF				0x00  // Default
#define RF_AUTOMODES_ENTER_FIFONOTEMPTY			0x20
#define RF_AUTOMODES_ENTER_FIFOLEVEL			0x40
#define RF_AUTOMODES_ENTER_CRCOK			0x60
#define RF_AUTOMODES_ENTER_PAYLOADREADY			0x80
#define RF_AUTOMODES_ENTER_SYNCADRSMATCH		0xA0
#define RF_AUTOMODES_ENTER_PACKETSENT			0xC0
#define RF_AUTOMODES_ENTER_FIFOEMPTY		 	0xE0

#define RF_AUTOMODES_EXIT_OFF				0x00  // Default
#define RF_AUTOMODES_EXIT_FIFOEMPTY	  		0x04
#define RF_AUTOMODES_EXIT_FIFOLEVEL	  		0x08
#define RF_AUTOMODES_EXIT_CRCOK				0x0C
#define RF_AUTOMODES_EXIT_PAYLOADREADY		  	0x10
#define RF_AUTOMODES_EXIT_SYNCADRSMATCH		  	0x14
#define RF_AUTOMODES_EXIT_PACKETSENT		  	0x18
#define RF_AUTOMODES_EXIT_RXTIMEOUT			0x1C

#define RF_AUTOMODES_INTERMEDIATE_SLEEP			0x00  // Default
#define RF_AUTOMODES_INTERMEDIATE_STANDBY		0x01
#define RF_AUTOMODES_INTERMEDIATE_RECEIVER		0x02
#define RF_AUTOMODES_INTERMEDIATE_TRANSMITTER           0x03


// RegFifoThresh
#define RF_FIFOTHRESH_TXSTART_FIFOTHRESH		0x00
#define RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY		0x80  // Default

// RegPacketConfig2
#define RF_PACKET2_RXRESTARTDELAY_1BIT			0x00  // Default
#define RF_PACKET2_RXRESTARTDELAY_2BITS			0x10
#define RF_PACKET2_RXRESTARTDELAY_4BITS	  		0x20
#define RF_PACKET2_RXRESTARTDELAY_8BITS		  	0x30
#define RF_PACKET2_RXRESTARTDELAY_16BITS		0x40
#define RF_PACKET2_RXRESTARTDELAY_32BITS  		0x50
#define RF_PACKET2_RXRESTARTDELAY_64BITS	  	0x60
#define RF_PACKET2_RXRESTARTDELAY_128BITS		0x70
#define RF_PACKET2_RXRESTARTDELAY_256BITS 		0x80
#define RF_PACKET2_RXRESTARTDELAY_512BITS	  	0x90
#define RF_PACKET2_RXRESTARTDELAY_1024BITS		0xA0
#define RF_PACKET2_RXRESTARTDELAY_2048BITS		0xB0
#define RF_PACKET2_RXRESTARTDELAY_NONE			0xC0
#define RF_PACKET2_RXRESTART			      	0x04

#define RF_PACKET2_AUTORXRESTART_ON			0x02  // Default
#define RF_PACKET2_AUTORXRESTART_OFF			0x00

#define RF_PACKET2_AES_ON				0x01
#define RF_PACKET2_AES_OFF				0x00  // Default


// RegTemp1
#define RF_TEMP1_MEAS_START				0x08
#define RF_TEMP1_MEAS_RUNNING				0x04

#ifndef RFM_CLK_OUTPUT
    #error RFM_CLK_OUTPUT must be defined to 0 or 1
#endif

#define SPI_WRITE8(reg,data)                            rfm_spi_write8(reg,data)
							// Enable Oscillator and 
#define RFM69_TX_ON_PRE()                                 SPI_WRITE8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_SYNTHESIZER )   

							// Enable Transmitter and Enable PacketSent-Interrupt
#define RFM69_TX_ON()                                     do{ SPI_WRITE8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_TRANSMITTER );\
							  SPI_WRITE8( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 ); } while (0)

                                                        // Enable Receiver and Enable Payload Ready Interrupt
#define RFM69_RX_ON()                                     do { SPI_WRITE8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_RECEIVER );\
							  SPI_WRITE8( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); } while(0)
  	
							// Enable Receiver, DIO0=Payload Ready, DIO1=FifoNotEmpty, DIO2=DataOutput, DIO3=SyncAddress
							// Use DIO1 as PCINT-InterruptSource in OOK Mode
#define RFM_RX_OOK_ON()                                 do { SPI_WRITE8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_RECEIVER );\
							  SPI_WRITE8( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01  | RF_DIOMAPPING1_DIO1_10 | RF_DIOMAPPING1_DIO2_01 | RF_DIOMAPPING1_DIO3_10); } while (0)

#if RFM_CLK_OUTPUT > 0
    #define RFM69_RXTX_OFF()                                   SPI_WRITE8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY )
#else
    #define RFM69_RXTX_OFF()                              SPI_WRITE8( REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_SLEEP ) 
#endif

