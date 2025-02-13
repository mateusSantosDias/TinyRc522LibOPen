#include<TinyRC522.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

  Tag::Tag(RC522* p_rc522):
    m_p_rc522(p_rc522)
{   
}

// Tag private functions
void Tag::initialize(){

   
    for(uint8_t i=0; i<7; i++) m_uid[i]=0x00;
    m_uid_size=0;
}

bool Tag::read(uint8_t* buffer,uint16_t buffer_size,uint8_t* key){

    
    if ((!buffer)||(buffer_size==0)||(buffer_size>TAG_MAX_SIZE)||(buffer_size%16)) return false;

   
    uint8_t factory_key[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (key==NULL) key=factory_key;

    
    uint16_t buffer_index=0;
    bool sector_authenticated=false;
    for(uint8_t block_address=1; block_address<64; block_address++)
    {
      
      if ((block_address%4)==3) sector_authenticated=false;
      else
      {
        
        if (!sector_authenticated)
        {
          uint8_t auth_command[12]={0x60,(uint8_t)((block_address/4)*4+3),key[0],key[1],key[2],key[3],key[4],key[5],m_uid[0],m_uid[1],m_uid[2],m_uid[3]};
          if (!m_p_rc522->authenticate(auth_command,12)) return false;
          sector_authenticated=true;
        }

        
        uint8_t read_command[4]={0x30,block_address,0x00,0x00};
        if (!m_p_rc522->write(read_command,4)) return false;

        
        uint8_t block_buffer[RC522_BLOCK_SIZE];
        uint8_t block_buffer_size=RC522_BLOCK_SIZE;
        if (!m_p_rc522->read(block_buffer,block_buffer_size)) return false;

        
        memcpy(&buffer[buffer_index],block_buffer,16);

        
        buffer_index+=16;
        if (buffer_index>=buffer_size) return true;
      }
    }

    return true;
  }

 
bool Tag::write(uint8_t* buffer,uint16_t buffer_size,uint8_t* key){
    
    if ((!buffer)||(buffer_size==0)||(buffer_size>TAG_MAX_SIZE)||(buffer_size%16)) return false;

    uint8_t factory_key[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (key==NULL) key=factory_key;

    uint16_t buffer_index=0;
    bool sector_authenticated=false;
    for(uint8_t block_address=1; block_address<64; block_address++)
    {

      if ((block_address%4)==3) sector_authenticated=false;
      else
      {
        if (!sector_authenticated)
        {
          uint8_t auth_command[12]={0x60,(uint8_t)((block_address/4)*4+3),key[0],key[1],key[2],key[3],key[4],key[5],m_uid[0],m_uid[1],m_uid[2],m_uid[3]};
          if (!m_p_rc522->authenticate(auth_command,12)) return false;
          sector_authenticated=true;
        }

        uint8_t write_command[4]={0xA0,block_address,0x00,0x00};
        if (!m_p_rc522->write(write_command,4)) return false;

        
        uint8_t block_buffer[RC522_BLOCK_SIZE];
        memcpy(block_buffer,&buffer[buffer_index],16);

        if (!m_p_rc522->write(block_buffer,RC522_BLOCK_SIZE)) return false;

        
        buffer_index+=16;
        if (buffer_index>=buffer_size) return true;
      }
    }
    
    return true;
}

bool Tag::release(){
    
    uint8_t halta_command[2]={0x50,0x00};
    return m_p_rc522->write(halta_command,sizeof(halta_command),0,false);
}


void Tag::add_uid_byte(uint8_t uid_byte){
    
    
    m_uid[m_uid_size++]=uid_byte;
}


void RC522::setup_spi(){
    
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SPI1);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO4|GPIO5|GPIO7);
    
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                  GPIO6);
    
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable(SPI1);
}

// RC522 public functions
void RC522::initialize(){
    
     setup_spi();

     spi_write_register(CommandReg, 0x0F);
     for (int i = 0; i < 800000; i++) __asm__("nop");

     spi_write_register(TModeReg, 0x80);
     spi_write_register(TPrescalerReg, 0xA9);
     spi_write_register(TReloadRegH,0x00);
     spi_write_register(TReloadRegL,0xF0);

     spi_write_register(TxASKReg,0x40);

      
     spi_clear_register_bits(CollReg,0x80);

    
     spi_set_register_bits(TxControlReg,0x03);
     
     m_tag.initialize();
     
}

void RC522::spi_write_register(uint8_t address, uint8_t value){
     
     address &= 0x7E;
     
     spi_send(SPI1, address);
     spi_send(SPI1, value);
}

  
uint32_t Tag::get_uid(){

    
    return (m_uid[3]<<24)|(m_uid[2]<<16)|(m_uid[1]<<8)|(m_uid[0]<<0);
}

uint16_t RC522::spi_read_register(uint8_t address){
     
     uint8_t spi_buffer[2]={(uint8_t) (address|0x80), 0x00};
     spi_write(SPI1, spi_buffer[0]);
     return  spi_read(SPI1);
}

void RC522::set_antenna_gain(uint8_t gain){

    if (gain){

        spi_write_register(RFCfgReg, gain);
        spi_set_register_bits(TxControlReg, 0x03);       
        
    } else {

        spi_clear_register_bits(TxControlReg, 0x03);
    }
}

void RC522::spi_set_register_bits(uint8_t address,uint8_t mask){

    
    spi_write_register(address,spi_read_register(address)|mask);
}


void RC522::spi_clear_register_bits(uint8_t address,uint8_t mask){


    spi_write_register(address,spi_read_register(address)&(~mask));
}

Tag* RC522::get_tag(){

    m_tag.initialize();\

    spi_clear_register_bits(Status2Reg, 0x08);

    uint8_t wupa_command[1] = {0x52};

    if(write(wupa_command, 1, 7, false)){

        uint8_t atqa[2] = {0x00, 0x00};
        uint8_t atqa_size = 2;

        if(read(atqa, atqa_size, false)){

            if((atqa_size==2)&&(atqa[0]==0x04)&&(atqa[1]==0x00)){

                uint8_t anticollision_cl1_command[2]={0x93, 0x20};
                if(write(anticollision_cl1_command, 2, 0, false)){

                    uint8_t uid[5];
                    uint8_t uid_size=5;
                    if(read(uid, uid_size, false)){
                        if((uid[0]^uid[1]^uid[2]^uid[3]) == uid[4]){
                            for(uint8_t i=0; i<4; i++) m_tag.add_uid_byte(uid[i]);

                            uint8_t select_cl1_command[9] = {0x93,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                            memcpy(&select_cl1_command[2], uid, 5);
                            if(write(select_cl1_command, 9)){

                                uint8_t sak[3];
                                uint8_t sak_size=3;

                                if(read(sak, sak_size)){

                                    if(sak[0]==0x08) return &m_tag;
                                }
                            }
                            
                        }
                    }
                }
            
            }
        }
    }
}


void RC522::spi_read_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size){

    if((!buffer) || (!buffer_size)) return;

    uint8_t spi_buffer[1+buffer_size];

    memset(spi_buffer, (uint8_t) (address | 0x80), 1+buffer_size);
    spi_write(SPI1, spi_buffer[0]);

    memcpy(buffer, &spi_buffer[1], buffer_size);
}

// RC522 private functions

bool RC522::read(uint8_t* buffer,uint8_t& buffer_size,bool control_crc_a){

    uint8_t bytes_in_fifo = spi_read_register(FIFOLevelReg);

    if((!buffer) || (!buffer_size) || (bytes_in_fifo > buffer_size)) return false;

    spi_read_register(FIFODataReg, buffer, bytes_in_fifo);
    buffer_size = bytes_in_fifo;

    uint8_t valid_bits = spi_read_register(ControlReg)&0x07;

    if((bytes_in_fifo==1)&&(valid_bits!=0x04)) return false;
    if((bytes_in_fifo<2)&&(valid_bits!=0x00)) return false;

    if(control_crc_a){

       uint8_t crc_a[2];
       calculate_crc_a(buffer, buffer_size-2,crc_a);
       if((buffer[buffer_size-2]!=crc_a[0])||
           (buffer[buffer_size-1]!=crc_a[1]))return false;
    }

    return true;
}
void RC522::spi_write_register(uint8_t address,uint8_t* buffer,uint8_t buffer_size){

     address &= 0x7E;  

    spi_send(SPI1, address);  

    for (uint8_t i = 0; i < buffer_size; i++) {
        spi_send(SPI1, buffer[i]);  
    }
}

bool RC522::write(uint8_t* buffer,uint8_t buffer_size,uint8_t last_byte_size,bool add_crc_a){

    if((!buffer) || (!buffer_size)) return false;

    if(add_crc_a) calculate_crc_a(buffer, buffer_size-2, &buffer[buffer_size-2]);

    spi_write_register(CommandReg,0x00);                    
    spi_write_register(ComIrqReg,0x7F);                     
    spi_write_register(FIFOLevelReg,0x80);                 
    spi_write_register(FIFODataReg,buffer,buffer_size);     
    spi_write_register(BitFramingReg,last_byte_size&0x07);  
    spi_write_register(CommandReg,0x0C);                    
    spi_set_register_bits(BitFramingReg,0x80); 

    uint32_t timeout=pdMS_TO_TICKS(36);
    uint8_t counter;
    while (1){

        uint16_t irq_bits = spi_read_register(ComIrqReg);
        if((irq_bits & 0x03) || (counter > timeout)) return false;
        if(irq_bits & 0x30) return true;
        
        counter++;
        //vTaskDelay(pdMS_TO_TICKS(1));
        for (int i = 0; i < 800000; i++) __asm__("nop");
    }
    
}

void RC522::calculate_crc_a(uint8_t* buffer,uint8_t buffer_size,uint8_t* result){

    if(!result) return;

    uint32_t crc=0x6363;

    do{
        
        uint8_t data_byte =* buffer++;
        data_byte = (data_byte^((uint8_t) (crc&0x00FF)));
        data_byte = (data_byte^(data_byte<<4));
        crc=(crc>>8)^((uint32_t) (data_byte<<8))^((uint32_t)(data_byte<<3))^((uint32_t) (data_byte>>4));
        
    } while (--buffer_size);

    result[0]=(uint8_t) (crc&0x00FF);
    result[1]=(uint8_t) ((crc>>8)&0x00FF);
    
}

  bool RC522::authenticate(uint8_t* buffer,uint8_t buffer_size)
  {
    // Exit on wrong buffer
    if ((!buffer)||(buffer_size==0)) return false;

    // Initialize RC522 module
    spi_write_register(CommandReg,0x00);                    // Stop any active command
    spi_write_register(ComIrqReg,0x7F);                     // Clear all IRQs
    spi_write_register(FIFOLevelReg,0x80);                  // Initialize FIFO
    spi_write_register(FIFODataReg,buffer,buffer_size);     // Write data to send in the FIFO
    spi_write_register(BitFramingReg,0x00);                 // Set bit adjustments
    spi_write_register(CommandReg,0x0E);                    // MFAuthent command

    while((spi_read_register(ComIrqReg)) != true);

    return true;
  }