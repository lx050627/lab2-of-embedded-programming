/********************************************************
 Name          : main.c
 Author        : Nomen Nescio
 Copyright     : Not really
 Description   : EVK1100 template
 **********************************************************/

// Include Files
//#include "hmatrix.h"
#include "board.h"
#include "compiler.h"
#include "sdramc.h"
#include "pm.h"
#include <avr32/io.h>
#include "pdca.h"
#include "sd_mmc_spi.h"
#include "spi.h"
#include "conf_sd_mmc_spi.h"
#include "mt48lc16m16a2tg7e.h"
#include "print_funcs.h"
#include "intc.h"
#include "gpio.h"
//#include "adc.h"

#define CPU_HZ 66000000
#define PBA_HZ 16500000
#define AVR32_PDCA_CHANNEL_SPI_RX 0 // In the example we will use the pdca channel 0.
#define AVR32_PDCA_CHANNEL_SPI_TX 1 // In the example we will use the pdca channel 1.
#define SECTOR_SIZE 512
volatile Bool end_of_transfer;
volatile avr32_pdca_channel_t* pdca_channeltx ;
volatile unsigned char *sdram = SDRAM;
//unsigned long sdram_size = SDRAM_SIZE >> 2;



void wait()
{
  volatile int i;
  for(i = 0 ; i < 5000; i++);
}

static void pdca_int_handler(void)
{
  print_dbg("\nINT-Transfer completed");
	// Disable all interrupts.
  Disable_global_interrupt();

  // Disable interrupt channel.
  pdca_disable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_TX);

  if(sd_mmc_spi_write_close_PDCA()== OK)//unselects the SD/MMC memory.
  {
	  wait();
  // Disable unnecessary channel
      pdca_disable(AVR32_PDCA_CHANNEL_SPI_TX);
 // pdca_disable(AVR32_PDCA_CHANNEL_SPI_RX);

     // Enable all interrupts.
      Enable_global_interrupt();
     print_dbg("\nResponse received");
  }
  else
  {
	  print_dbg("\nResponse NOT received");
  }

  end_of_transfer = TRUE;

}

void sd_mmc_resources_init(void)
{
   //GPIO pins used for SD/MMC interface
  static const gpio_map_t SD_MMC_SPI_GPIO_MAP =
  {
    {SD_MMC_SPI_SCK_PIN,  SD_MMC_SPI_SCK_FUNCTION },  // SPI Clock.
    {SD_MMC_SPI_MISO_PIN, SD_MMC_SPI_MISO_FUNCTION},  // MISO.
    {SD_MMC_SPI_MOSI_PIN, SD_MMC_SPI_MOSI_FUNCTION},  // MOSI.
    {SD_MMC_SPI_NPCS_PIN, SD_MMC_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
  };

	//SPI options.
	  spi_options_t spiOptions =
	  {
	    .reg          = SD_MMC_SPI_NPCS,
	    .baudrate     = PBA_HZ,  // Defined in conf_sd_mmc_spi.h.
	    .bits         = SD_MMC_SPI_BITS,          // Defined in conf_sd_mmc_spi.h.
	    .spck_delay   = 0,
	    .trans_delay  = 0,
	    .stay_act     = 1,
	    .spi_mode     = 0,
	    .modfdis      = 1
	  };

  // Assign I/Os to SPI.
  gpio_enable_module(SD_MMC_SPI_GPIO_MAP,sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));

  // Initialize as master.
  spi_initMaster(SD_MMC_SPI, &spiOptions);

  // Set SPI selection mode: variable_ps, pcs_decode, delay.
  spi_selectionMode(SD_MMC_SPI, 0, 0, 0);

  // Enable SPI module.
  spi_enable(SD_MMC_SPI);

  // Initialize SD/MMC driver with SPI clock (PBA).
  sd_mmc_spi_init(spiOptions, PBA_HZ);
}

void local_pdca_init(void)
{
  // this PDCA channel is used for data reception from the SPI
  /*pdca_channel_options_t pdca_options_SPI_RX ={ // pdca channel options

    .addr = ram_buffer,
    // memory address. We take here the address of the string dummy_data. This string is located in the file dummy.h

    .size = 512,                              // transfer counter: here the size of the string
    .r_addr = NULL,                           // next memory address after 1st transfer complete
    .r_size = 0,                              // next transfer counter not used here
    .pid = AVR32_PDCA_CHANNEL_USED_RX,        // select peripheral ID - data are on reception from SPI1 RX line
    .transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer: 8,16,32 bits
  };*/

  // this channel is used to activate the clock of the SPI by sending a dummy variables
  pdca_channel_options_t pdca_options_SPI_TX ={ // pdca channel options

    .addr = (void *)sdram,              // memory address.
                                              // We take here the address of the string dummy_data.
                                              // This string is located in the file dummy.h
    .size = 512,                              // transfer counter: here the size of the string
    .r_addr = NULL,                           // next memory address after 1st transfer complete
    .r_size = 0,                              // next transfer counter not used here
    .pid = 16,        // select peripheral ID - data are on reception from SPI1 RX line
    .transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer: 8,16,32 bits
  };

  // Init PDCA transmission channel
  pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_TX, &pdca_options_SPI_TX);

  // Init PDCA Reception channel
 // pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_RX, &pdca_options_SPI_RX);

  //\brief Enable pdca transfer interrupt when completed
   INTC_register_interrupt(&pdca_int_handler, AVR32_PDCA_IRQ_1, AVR32_INTC_INT1);  // pdca_channel_spi1_RX = 0

}

int main(void)
{
	//use power manger driver for setting the frequency
	pm_freq_param_t System_Clock = {
		    .cpu_f = CPU_HZ,
		    .pba_f = PBA_HZ,
		    .osc0_f = FOSC0,
		    .osc0_startup = OSC0_STARTUP
		};
	pm_configure_clocks(&System_Clock);

	// Initialize the debug USART module.
	init_dbg_rs232(PBA_HZ);

	//SDRAM address
	// Initialize the external SDRAM chip.
	sdramc_init(CPU_HZ);
	print_dbg("\nSDRAM initialized");
	//for detail, see SDRAM example

	// Setting EBI slave to have fixed default master
	AVR32_HMATRIX.SCFG[AVR32_HMATRIX_SLAVE_EBI].defmstr_type=AVR32_HMATRIX_DEFMSTR_TYPE_FIXED_DEFAULT;

	//Setting EBI slave to have PDCA as a master
	AVR32_HMATRIX.SCFG[AVR32_HMATRIX_SLAVE_EBI].fixed_defmstr=AVR32_HMATRIX_MASTER_PDCA;


	// Initialize Interrupt Controller
	  INTC_init_interrupts();

	  // Enable all interrupts.
	   Enable_global_interrupt();

	// Initialize SD/MMC driver resources: GPIO, SPI and SD/MMC.
	   sd_mmc_resources_init();

	// Wait for a card to be inserted
//	  while ( sd_mmc_spi_mem_check() != OK );
	  print_dbg("\nCard detected!");

	  // Read Card capacity
	  sd_mmc_spi_get_capacity();
	  print_dbg("Capacity = ");
	  print_dbg_ulong(capacity >> 20);
	  print_dbg(" MBytes");

	  // Initialize PDCA controller before starting a transfer
	  local_pdca_init();
	  print_dbg("\nPDCA initialized");
	  /*Pre-write the following dummy data into the whole 32MB SDRAM:
	  Data byte sequence start from 0x00 increment to 0xFF and repeat again.
	  */
	  gpio_clr_gpio_pin(LED1_GPIO);

	  U32 i;
      U8 j;
	  for (i = 0, j = 0; i < SDRAM_SIZE; i++)
	  {
	      sdram[i] = j;
	      j++;

	   }
	  gpio_clr_gpio_pin(LED3_GPIO);
	  print_dbg("prewrite SDRAM done!!");

	  end_of_transfer = FALSE;
	  for(i = 1; i <=SDRAM_SIZE/SECTOR_SIZE; i++)
	  {
	     pdca_load_channel( AVR32_PDCA_CHANNEL_SPI_TX,(void *)sdram,512);

	     // open sector number j
	     if(sd_mmc_spi_write_open_PDCA (i) == OK)
	     {
	    	 print_dbg("\n 512 Bytes of Transfer Number ");
	    	 print_dbg_ulong(i);
	    	// print_dbg(" :\n");
	    	 //print_dbg("at first remaining size:");
	    	// print_dbg_ulong(pdca_get_load_size(AVR32_PDCA_CHANNEL_SPI_TX)) ;
	       //spi_write(SD_MMC_SPI,0x11);
	        pdca_enable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_TX);
	      // pdca_channelrx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_RX); // get the correct PDCA channel pointer
	        pdca_channeltx =(volatile avr32_pdca_channel_t*)pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_TX); // get the correct PDCA channel pointer
	      // pdca_channelrx->cr = AVR32_PDCA_TEN_MASK; // Enable RX PDCA transfer first
	       pdca_channeltx->cr = AVR32_PDCA_TEN_MASK; // and TX PDCA transfer
	       print_dbg("/");
	       print_dbg_ulong(pdca_get_load_size(AVR32_PDCA_CHANNEL_SPI_TX)) ;
	      // print_dbg("sending.. ");

	      // U32 status=pdca_get_channel_status(AVR32_PDCA_CHANNEL_SPI_TX);
	       //print_dbg("channel status:");
	      // print_dbg_ulong(status);

	       while(!end_of_transfer)
	       {

	       }
	       sdram+=512;

	     }
	     else
	     {
	       print_dbg("\n! Unable to open memory \n");
	     }
	   }

	  print_dbg("\nEnd of the lab2.\n");

	  while(1){
		  gpio_clr_gpio_pin(LED5_GPIO);

	  }

	return 0;
}
