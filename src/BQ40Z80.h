#ifndef BQ40Z80_H
#define BQ40Z80_H

#include <Arduino.h>
#include <Wire.h>

// Note: The BQ40Z80 default 7-bit address is 0x0B
#define BQ40Z80_DEFAULT_ADDR 0x0B

// --- Standard SBS Command Codes ---
#define BQ40Z80_CMD_TEMPERATURE         0x08
#define BQ40Z80_CMD_VOLTAGE             0x09
#define BQ40Z80_CMD_CURRENT             0x0A
#define BQ40Z80_CMD_AVG_CURRENT         0x0B
#define BQ40Z80_CMD_SOC                 0x0D // RelativeStateOfCharge
#define BQ40Z80_CMD_REMAINING_CAPACITY  0x0F
#define BQ40Z80_CMD_FULL_CHARGE_CAP     0x10
#define BQ40Z80_CMD_CYCLE_COUNT         0x17
#define BQ40Z80_CMD_CELL_VOLTAGE_4      0x3C // Cell 7
#define BQ40Z80_CMD_CELL_VOLTAGE_3      0x3D // Cell 6
#define BQ40Z80_CMD_CELL_VOLTAGE_2      0x3E // Cell 5
#define BQ40Z80_CMD_CELL_VOLTAGE_1      0x3F // Cell 4

// --- Manufacturer Access (MAC) Commands ---
#define BQ40Z80_MAC_MANUFACTURER_ACCESS 0x00
#define BQ40Z80_MAC_MANUFACTURER_DATA   0x23
#define BQ40Z80_MAC_BLOCK_ACCESS        0x44

// Specific MAC Commands
#define BQ40Z80_MAC_DEVICE_TYPE         0x0001
#define BQ40Z80_MAC_CHEM_ID             0x0006
#define BQ40Z80_MAC_DASTATUS1           0x0071 // Cells 1-4
#define BQ40Z80_MAC_DASTATUS3           0x007B // Cells 5-7
#define BQ40Z80_MAC_SEAL_DEVICE         0x0030

// MAC Commands for FET Control
#define BQ40Z80_MAC_CHGFET_TOGGLE       0x001F
#define BQ40Z80_MAC_DSGFET_TOGGLE       0x0020
#define BQ40Z80_MAC_FET_CONTROL         0x0022 // Resets safety latches

// MAC Commands for Diagnostics
#define BQ40Z80_MAC_SAFETY_ALERT        0x0050
#define BQ40Z80_MAC_SAFETY_STATUS       0x0051
#define BQ40Z80_MAC_PF_ALERT            0x0052
#define BQ40Z80_MAC_PF_STATUS           0x0053
#define BQ40Z80_MAC_OPERATION_STATUS    0x0054
#define BQ40Z80_MAC_CHARGING_STATUS     0x0055
#define BQ40Z80_MAC_GAUGING_STATUS      0x0056
#define BQ40Z80_MAC_MANUFACTURING_STATUS 0x0057 
#define BQ40Z80_MAC_DEVICE_RESET        0x0041


// MAC Commands for Lifetime Data
#define BQ40Z80_MAC_LIFETIME_BLOCK_1    0x0060
#define BQ40Z80_MAC_LIFETIME_BLOCK_2    0x0061
#define BQ40Z80_MAC_LIFETIME_RESET      0x0028


// --- Default Security Keys (for Unseal/Full Access) ---
#define BQ40Z80_UNSEAL_KEY1             0x0414
#define BQ40Z80_UNSEAL_KEY2             0x3672
#define BQ40Z80_FULLACCESS_KEY1         0xFFFF
#define BQ40Z80_FULLACCESS_KEY2         0xFFFF


// --- Data Flash (DF) Addresses ---
// General Config
#define BQ40Z80_DF_MFG_STATUS_INIT      0x4800 // Contains FET_EN (Bit 4)
#define BQ40Z80_DF_DESIGN_CAP           0x4B69
#define BQ40Z80_DF_CELL_CONFIG          0x4D05
#define BQ40Z80_DF_BALANCING_CONFIG     0x4B8C // <-- ADDED
#define BQ40Z80_DF_TEMP_ENABLE          0x4D01
#define BQ40Z80_DF_DA_CONFIG            0x4D03 
#define BQ40Z80_DF_UPDATE_STATUS        0x44D8

// Gauging Parameters
#define BQ40Z80_DF_TERM_VOLTAGE         0x4A51
#define BQ40Z80_DF_CHG_TERM_TAPER_CURR  0x4CF3
#define BQ40Z80_DF_LOAD_MODE            0x4BA5
#define BQ40Z80_DF_SLEEP_CURRENT        0x4AF8
#define BQ40Z80_DF_VOLTAGE_TIME         0x4AFF
#define BQ40Z80_DF_CURRENT_TIME         0x4B00


// System Interface
#define BQ40Z80_DF_PIN_CONFIG           0x4AD3
#define BQ40Z80_DF_LED_CONFIG           0x4B0C

// CUV/COV/OCC/OCD
#define BQ40Z80_DF_CUV_THRESH           0x4BC2
#define BQ40Z80_DF_CUV_DELAY            0x4BC4
#define BQ40Z80_DF_CUV_RECOVERY         0x4BC5
#define BQ40Z80_DF_COV_THRESH_LOW       0x4BCC 
#define BQ40Z80_DF_COV_DELAY            0x4BD6
#define BQ40Z80_DF_COV_RECOVERY_LOW     0x4BD7 
#define BQ40Z80_DF_OCC1_THRESH          0x4BE4
#define BQ40Z80_DF_OCC1_DELAY           0x4BE6
#define BQ40Z80_DF_OCD1_THRESH          0x4BED
#define BQ40Z80_DF_OCD1_DELAY           0x4BEF

// Thermistor Coefficients
#define BQ40Z80_DF_A1_COEF              0x4848 
#define BQ40Z80_DF_A2_COEF              0x484A 
#define BQ40Z80_DF_A3_COEF              0x484C 
#define BQ40Z80_DF_A4_COEF              0x484E 
#define BQ40Z80_DF_A5_COEF              0x4850 
#define BQ40Z80_DF_B1_COEF              0x4852 
#define BQ40Z80_DF_B2_COEF              0x4854 
#define BQ40Z80_DF_B3_COEF              0x4856 
#define BQ40Z80_DF_B4_COEF              0x4858 

// AFE Protection Thresholds
#define BQ40Z80_DF_AOLD_THRESH          0x4D08
#define BQ40Z80_DF_ASCC_THRESH          0x4D09
#define BQ40Z80_DF_ASCD1_THRESH         0x4D0A
#define BQ40Z80_DF_ASCD2_THRESH         0x4D0B

// Temperature Protection Thresholds (0.1K)
#define BQ40Z80_DF_OT_CHARGE_THRESH     0x4C05
#define BQ40Z80_DF_OT_CHARGE_RECOV      0x4C08
#define BQ40Z80_DF_OT_DSG_THRESH        0x4C0A
#define BQ40Z80_DF_OT_DSG_RECOV         0x4C0D 
#define BQ40Z80_DF_UT_CHARGE_THRESH     0x4C14
#define BQ40Z80_DF_UT_CHARGE_RECOV      0x4C17
#define BQ40Z80_DF_UT_DSG_THRESH        0x4C19
#define BQ40Z80_DF_UT_DSG_RECOV         0x4C1C


// --- Config Constants ---
#define BQ40Z80_MFG_GAUGE_EN_BIT (1 << 3)
#define BQ40Z80_MFG_FET_EN_BIT (1 << 4) // Bit 4 in 0x4800 (Manufacturing Status Init)
#define BQ40Z80_DA_CONFIG_NR_BIT (1 << 2) 
#define BQ40Z80_OPSTATUS_SEC1_BIT (1UL << 9) // Security Mode Bit 1
#define BQ40Z80_OPSTATUS_SEC0_BIT (1UL << 8) // Security Mode Bit 0


// --- User-Friendly Configuration Structs ---

struct CUVConfig {
  uint16_t threshold_mV; 
  uint8_t  delay_s;      
  uint16_t recovery_mV;  
};

struct COVConfig {
  uint16_t threshold_mV; 
  uint8_t  delay_s;      
  uint16_t recovery_mV;  
};

struct OCCConfig {
  uint16_t threshold_mA; 
  uint8_t  delay_s;      
};

struct OCDConfig {
  int16_t  threshold_mA; 
  uint8_t  delay_s;      
};

struct AFEConfig {
  uint8_t threshold_index; 
  uint8_t delay_index;     
};

struct TempConfig {
  int16_t threshold_C; 
  int16_t recovery_C;  
};

struct SleepConfig {
  int16_t sleep_current_mA; 
  uint8_t voltage_time_s; 
  uint8_t current_time_s; 
};

// --- NEW BALANCING CONFIG STRUCT ---
struct BalancingConfig {
  bool master_enable;      // Bit 0: CB
  bool external_balancing;   // Bit 1: CBM (true=external, false=internal)
  bool balance_at_rest;      // Bit 2: CBR
  bool use_weighted_dod_charge; // Bit 3: CB_CHG_DOD0EW
  bool use_weighted_dod_relax;  // Bit 4: CB_RLX_DOD0EW
  bool balance_in_sleep;     // Bit 5: CBS
};

// --- Structs for Lifetime Data Blocks ---

struct LifetimeDataBlock1 {
  // All voltages in mV
  uint16_t cell_max_voltage[7]; // 14 bytes
  uint16_t cell_min_voltage[7]; // 14 bytes
  uint16_t max_delta_cell_voltage; // 2 bytes
  // 30 bytes total
};

// This struct is now 25 bytes total
struct LifetimeDataBlock2 {
  // All units as per TRM
  int16_t max_charge_current;   // mA (2 bytes)
  int16_t max_discharge_current; // mA (2 bytes)
  int16_t max_avg_dsg_current;  // mA (2 bytes)
  int16_t max_avg_dsg_power;    // cW (2 bytes)
  int8_t  max_temp_cell;        // C  (1 byte)
  int8_t  min_temp_cell;        // C  (1 byte)
  int8_t  max_delta_cell_temp;  // C  (1 byte)
  int8_t  max_temp_int_sensor;  // C  (1 byte)
  int8_t  min_temp_int_sensor;  // C  (1 byte)
  int8_t  max_temp_fet;         // C  (1 byte)
  uint8_t num_shutdowns;        // (1 byte)
  uint8_t num_partial_resets;   // (1 byte)
  uint8_t num_full_resets;      // (1 byte)
  uint8_t num_wdt_resets;       // (1 byte)
  uint8_t cb_time_cell[7];      // 2-hour units (7 bytes)
  // TOTAL: 8 + 6 + 4 + 7 = 25 bytes
};


// --- Safety Status Flag Decoding ---
// Bit positions in the 32-bit SafetyStatus register (0x0051)
#define BQ40Z80_SAFETY_CUV  (1UL << 0)  // Cell Undervoltage
#define BQ40Z80_SAFETY_COV  (1UL << 1)  // Cell Overvoltage
#define BQ40Z80_SAFETY_OCC1 (1UL << 2)  // Overcurrent in Charge 1
#define BQ40Z80_SAFETY_OCC2 (1UL << 3)  // Overcurrent in Charge 2
#define BQ40Z80_SAFETY_OCD1 (1UL << 4)  // Overcurrent in Discharge 1
#define BQ40Z80_SAFETY_OCD2 (1UL << 5)  // Overcurrent in Discharge 2
#define BQ40Z80_SAFETY_AOLD (1UL << 6)  // Overload in Discharge
#define BQ40Z80_SAFETY_ASCC (1UL << 8)  // Short Circuit in Charge
#define BQ40Z80_SAFETY_ASCD (1UL << 10) // Short Circuit in Discharge
#define BQ40Z80_SAFETY_OTC  (1UL << 12) // Overtemperature in Charge
#define BQ40Z80_SAFETY_OTD  (1UL << 13) // Overtemperature in Discharge
#define BQ40Z80_SAFETY_OTF  (1UL << 16) // Overtemperature FET
#define BQ40Z80_SAFETY_UTC  (1UL << 26) // Undertemperature in Charge
#define BQ40Z80_SAFETY_UTD  (1UL << 27) // Undertemperature in Discharge

// Define the max number of flags we will decode
#define BQ40Z80_MAX_SAFETY_FLAGS 15 

// Array of names for decoding (must match order of defines above)
const String BQ40Z80_SAFETY_FLAG_NAMES[BQ40Z80_MAX_SAFETY_FLAGS] = {
  "CUV (Cell Undervoltage)",
  "COV (Cell Overvoltage)",
  "OCC1 (Overcurrent Charge 1)",
  "OCC2 (Overcurrent Charge 2)",
  "OCD1 (Overcurrent Discharge 1)",
  "OCD2 (Overcurrent Discharge 2)",
  "AOLD (Overload Discharge)",
  "ASCC (Short Circuit Charge)",
  "ASCD (Short Circuit Discharge)",
  "OTC (Overtemp Charge)",
  "OTD (Overtemp Discharge)",
  "OTF (Overtemp FET)",
  "UTC (Undertemp Charge)",
  "UTD (Undertemp Discharge)",
  "UNKNOWN (Other)" // Placeholder for bits not defined
};

const uint32_t BQ40Z80_SAFETY_FLAG_MASKS[BQ40Z80_MAX_SAFETY_FLAGS] = {
  BQ40Z80_SAFETY_CUV, BQ40Z80_SAFETY_COV, BQ40Z80_SAFETY_OCC1, BQ40Z80_SAFETY_OCC2,
  BQ40Z80_SAFETY_OCD1, BQ40Z80_SAFETY_OCD2, BQ40Z80_SAFETY_AOLD, BQ40Z80_SAFETY_ASCC,
  BQ40Z80_SAFETY_ASCD, BQ40Z80_SAFETY_OTC, BQ40Z80_SAFETY_OTD, BQ40Z80_SAFETY_OTF,
  BQ40Z80_SAFETY_UTC, BQ40Z80_SAFETY_UTD, 0
};


class BQ40Z80 {
  public:
    BQ40Z80(uint8_t address = BQ40Z80_DEFAULT_ADDR, TwoWire &wire = Wire);
    bool begin();

    // --- Standard SBS Commands ---
    float getVoltage();
    float getCurrent();
    float getAvgCurrent();
    float getTemperature();
    uint8_t getSOC(); 
    uint16_t getRemCapacity();
    uint16_t getFullCapacity();
    uint16_t getCycleCount();
    uint16_t getCellVoltage(uint8_t cell_index); // 1-based index (1-7)

    // --- Advanced MAC Commands ---
    uint16_t getDeviceType();
    uint16_t getChemID();
    bool getAllCellVoltages(uint16_t* cellVoltages_mV); // Expects 7 cells
    uint32_t getGaugingStatus();
    uint32_t getChargingStatus();
    uint32_t getOperationStatus(); 

    // --- Diagnostics (32-bit Reads) ---
    uint32_t getSafetyStatus();
    uint32_t getSafetyAlerts();
    uint32_t getPFStatus();
    
    // --- Diagnostic Helpers ---
    int decodeSafetyStatus(uint32_t status_flags, String* fault_buffer);
    bool resetProtectionStatus();
    bool isSealed(); 

    // --- Lifetime Data ---
    bool getLifetimeDataBlock1(LifetimeDataBlock1& data);
    bool getLifetimeDataBlock2(LifetimeDataBlock2& data);
    bool resetLifetimeData();

    // --- Configuration & Security ---
    bool unseal();
    bool enterFullAccess();
    bool seal();
    
    bool beginConfig();
    bool endConfig();

    // --- High-Level Configuration Functions ---
    bool setDesignCapacity(uint16_t capacity_mAh);
    bool setCellCount(uint8_t cellCount); // 1-7
    bool setThermistorConfig(bool ts1, bool ts2, bool ts3, bool ts4);
    bool enableGauging();
    bool setThermistorCoefficients(int16_t coefficients[9]);

    // --- Gauging & Calibration
    bool setUpdateStatus(uint8_t status);
    bool setTermVoltage(uint16_t voltage_mV);
    bool setChargeTermTaper(uint16_t current_mA);
    bool setLoadMode(uint8_t mode); // 0=CC, 1=CP

    // --- System & Interface
    bool setPinConfiguration(uint16_t config_word);
    bool setLEDConfiguration(uint16_t config_word);
    bool setSleepConfig(SleepConfig config);
    bool setNonRemovable(bool enable); 
    bool enableAutoFET(bool auto_control); // Control FET_EN bit 
    bool setCHGFETToggle();
    bool setDSGFETToggle();
    bool resetDevice();

    // --- Protection Configuration Functions ---
    bool setCUVConfig(CUVConfig config);
    bool setCOVConfig(COVConfig config);
    bool setOCC1Config(OCCConfig config);
    bool setOCD1Config(OCDConfig config);
    bool setBalancingConfig(BalancingConfig config); // <-- ADDED
    
    // Custom AFE Protection (using index values)
    bool setAOLDConfig(AFEConfig config);
    bool setASCCConfig(AFEConfig config);
    bool setASCD1Config(AFEConfig config);
    bool setASCD2Config(AFEConfig config);

    // Custom Temperature Protection
    bool setOTCConfig(TempConfig config);
    bool setOTDConfig(TempConfig config);
    bool setUTCConfig(TempConfig config);
    bool setUTDConfig(TempConfig config);


    // --- Low Level Access ---
    uint8_t readByte(uint8_t command); 
    uint16_t readWord(uint8_t command);
    bool writeWord(uint8_t command, uint16_t data);
    bool readMACBlock(uint16_t macCommand, uint8_t* buffer, uint8_t expectedLen); 
    
    // Using C++ standard types to avoid linker errors
    bool writeDFBlock(unsigned int address, unsigned char* data, unsigned char len);
    
    bool writeDFWord(uint16_t address, int16_t value);
    bool readDFWord(uint16_t address, int16_t& value); 
  private:
    uint8_t _deviceAddress;
    TwoWire *_wire;

    // --- Private Write Helpers ---
    bool writeDFByte(uint16_t address, uint8_t value);

};

#endif