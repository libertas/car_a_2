#include "flash.h"

u32 get_sector(u32 addr){
    u32 secotr = 0;
    else if((addr < ADDR_FLASH_SECTOR_2) && (addr >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((addr < ADDR_FLASH_SECTOR_3) && (addr >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((addr < ADDR_FLASH_SECTOR_4) && (addr >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((addr < ADDR_FLASH_SECTOR_5) && (addr >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((addr < ADDR_FLASH_SECTOR_6) && (addr >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((addr < ADDR_FLASH_SECTOR_7) && (addr >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((addr < ADDR_FLASH_SECTOR_8) && (addr >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((addr < ADDR_FLASH_SECTOR_9) && (addr >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((addr < ADDR_FLASH_SECTOR_10) && (addr >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((addr < ADDR_FLASH_SECTOR_11) && (addr >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
    sector = FLASH_Sector_11;  
  }

  return sector;
}
