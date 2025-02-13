#ifndef _TINY_RC522_H_
#define _TINY_RC522_H_
     /*Defines do Módulo, Inclusões e typedefs*/
     
     // Tag maximum size
     #define TAG_MAX_SIZE              752             // 47 blocks of 16 bytes

     // Antenna gain
     #define ANTENNA_GAIN_OFF          0x00            // Antenna off
     #define ANTENNA_GAIN_18_DB        0x28            // 18 dB
     #define ANTENNA_GAIN_23_DB        0x38            // 23 dB
     #define ANTENNA_GAIN_33_DB        0x48            // 33 dB
     #define ANTENNA_GAIN_38_DB        0x58            // 38 dB
     #define ANTENNA_GAIN_43_DB        0x68            // 43 dB
     #define ANTENNA_GAIN_48_DB        0x78            // 48 dB

     // Block size
     #define RC522_BLOCK_SIZE          18             // 16 bytes block + 2 bytes CRC_A

     // RC522 registers
     #define CommandReg                (0x01<<1)
     #define ComIrqReg                 (0x04<<1)
     #define ErrorReg                  (0x06<<1)
     #define Status2Reg                (0x08<<1)
     #define FIFODataReg               (0x09<<1)
     #define FIFOLevelReg              (0x0A<<1)
     #define ControlReg                (0x0C<<1)
     #define BitFramingReg             (0x0D<<1)
     #define CollReg                   (0x0E<<1)
     #define ModeReg                   (0x11<<1)
     #define TxModeReg                 (0x12<<1)
     #define RxModeReg                 (0x13<<1)
     #define TxControlReg              (0x14<<1)
     #define TxASKReg                  (0x15<<1)
     #define RFCfgReg                  (0x26<<1)
     #define TModeReg                  (0x2A<<1)
     #define TPrescalerReg             (0x2B<<1)
     #define TReloadRegH               (0x2C<<1)
     #define TReloadRegL               (0x2D<<1)
     #define VersionReg                (0x37<<1)

     #include<libopencm3/stm32/spi.h>
     #include<libopencm3/stm32/gpio.h>
     #include<libopencm3/stm32/rcc.h>

#endif
     /*Protóripos de Função e classes*/

     // RC522 class declaration
     class RC522;


     // Mifare 1K S50 (4-bytes UID) class definition
     class Tag
     {
         // Friend class access
         friend class RC522;



         // Private attributes
         private:

         // RC522 module pointer
         RC522* m_p_rc522;

         // UID
         uint8_t m_uid[7];
         uint8_t m_uid_size;



         // Public functions
         public:

         // Constructor
         Tag(RC522* p_rc522);

         // Get UID
         uint32_t get_uid();

         // Set/remove protection
         bool set_protection(uint8_t* new_key,uint8_t* key);
         bool remove_protection(uint8_t* key);

         // Read/write data
         bool read(uint8_t* buffer,uint16_t buffer_size,uint8_t* key);
         bool write(uint8_t* buffer,uint16_t buffer_size,uint8_t* key);

         // Release
         bool release();



         // Private functions
         private:

         // Initialize
         void initialize();

         // Add UID byte
         void add_uid_byte(uint8_t uid_byte);
};



// RC522 class definition
class RC522
{
     // Friend class access
     friend class Tag;



     // Private attributes
     private:

     // Tag
     Tag m_tag{this};



     // Public functions
     public:

     // Initialize
     void initialize();

     // Set antenna gain
     void set_antenna_gain(uint8_t gain);

     // Get tag
     Tag* get_tag();

     // Get module version
     uint8_t get_version();

     void setup_spi();



     // Private functions
     private:

     // Authenticate
     bool authenticate(uint8_t* buffer,uint8_t buffer_size);

     // Read/write data
     bool read(uint8_t* buffer,uint8_t& buffer_size,bool control_crc_a=true);
     bool write(uint8_t* buffer,uint8_t buffer_size,uint8_t last_byte_size=0,bool add_crc_a=true);

     // Calculate CRC_A
     void calculate_crc_a(uint8_t* buffer,uint8_t buffer_size,uint8_t* result);
 
     // SPI Set/clear register bits
     void spi_set_register_bits(uint8_t address,uint8_t mask);
     void spi_clear_register_bits(uint8_t address,uint8_t mask);

     // SPI Read register
     uint16_t spi_read_register(uint8_t address);
     void spi_read_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size);

     // SPI Write register
     void spi_write_register(uint8_t address,uint8_t value);
     void spi_write_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size);

     // SPI Select/release device
     void spi_select_device();
     void spi_release_device();
};


#ifdef DEF_TINY_RC522
    #define TINY_RC522 extern
#else
    #define TINY_RC522
#endif	/* DRIVER_LCD_H */
    /*Variáveis Globais*/