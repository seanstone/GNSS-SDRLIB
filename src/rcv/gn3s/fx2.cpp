/*
 * Code from: USRP - Universal Software Radio Peripheral (GNU Radio)
 *
 * Initial modifications by:
 *
 * Stephan Esterhuizen, Aerospace Engineering Sciences
 * University of Colorado at Boulder
 * Boulder CO, USA
 *
 * Further modifications for use with the SiGe USB module to accompany
 * the textbook: "A Software-Defined GPS and Galileo Receiver: A
 * Single-Frequency Approach" by Kai Borre, Dennis Akos, et.al. by:
 *
 * Marcus Junered, GNSS Research Group
 * Lulea University of Technology
 * Lulea, Sweden
 *
 * Further review and modifications of user interface has been made by:
 *
 * Jonas Lindstrom
 * Lulea University of Technology
 * Lulea, Sweden
 *
 * http://ccar.colorado.edu/gnss
 *
 *  Furthur modifications integrating 4120 devices have been made by:
 *
 *  Marcus Wilkerson
 *  University of Colorado, Boulder
 *  Boulder, CO, USA
 *
 *  Some minor changes were made by
 *
 *  Oscar Isoz
 *  University of Colorado, Boulder
 *  Boulder, CO, USA
 *
 * Change Documentation (2/2008 - 5/2008)
 * --------------------------------------
 * - MW - Added PID array variable for SiGe 4120 GPS Front End
 * - MW - Fixed a bug if LIBUSB found more devices on the bus and gave
 *        each device a different index number. It was hardcoded to "1" before.
 * - MW - Added check for 4120 module in usb_fx2_find fcn
 * - MW - If 4120, use I/Q values of -1 and +1 instead of -1,-3,+1,+3 as with
 *        4110 modules. This is in main().
 * - OI - Changed the user interface
 *
 * ---------------------------------------------------------------------
 *
 * GN3S - GNSS IF Streamer for Windows
 * Copyright (C) 2006 Marcus Junered
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Also, part of the code is taken from "ezusb.c"
 * Copyright © 2001 Stephen Williams (steve@icarus.com)
 * Copyright © 2001-2002 David Brownell (dbrownell@users.sourceforge.net)
 * Copyright © 2008 Roger Williams (rawqux@users.sourceforge.net)
 * Copyright © 2012 Pete Batard (pete@akeo.ie)
 * Copyright © 2013 Federico Manzan (f.manzan@gmail.com)

 *  Also part of Cannelloni/fx2pipe code https://github.com/jhol/cannelloni was added here by iliasam in 2023

 */
#include "sdr.h"
#include "fx2.h"

using namespace std;

#define RW_INTERNAL					0xA0	/* hardware implements this one */
#define RW_MEMORY					0xA3
#define RETRY_LIMIT					5
#define LIBUSB_ENDPOINT_OUT			0x00
#define LIBUSB_REQUEST_TYPE_VENDOR	(0x02 << 5)
#define LIBUSB_RECIPIENT_DEVICE		0x00

typedef enum {
	_undef = 0,
	internal_only,		/* hardware first-stage loader */
	skip_internal,		/* first phase, second-stage loader */
	skip_external		/* second phase, second-stage loader */
} ram_mode;

struct ram_poke_context {
	usb_dev_handle *device;
	ram_mode mode;
	size_t total, count;
};

Fx2_dev::Fx2_dev()
{
	int fx2_vid[] = { 0x1781,0x1781,0x1781,0x1781,0x04B4};
    int fx2_pid[]={0x0b38,0x0b39,0x0b3a,0x0b3f,0x8613};
    fx2_port=0;
    fx2_conf = new Fx2_c();
    agc_buf= new char[GN3S_AGC_BUFFSIZE];
}

Fx2_dev::Fx2_dev(int i)
{
	fx2_vid[0] = 0x1781;
	fx2_vid[1] = 0x1781;
	fx2_vid[2] = 0x1781;
	fx2_vid[3] = 0x1781;
	fx2_vid[4] = 0x04B4;//no eeprom

    fx2_pid[0]=0x0b38;
    fx2_pid[1]=0x0b39;
    fx2_pid[2]=0x0b3a;
    fx2_pid[3]=0x0b3f;
	fx2_pid[4] = 0x8613;//no eeprom

    fx2_port=i;
    fx2_conf = new Fx2_c();
    agc_buf= new char[GN3S_AGC_BUFFSIZE];
}

Fx2_dev::~Fx2_dev()
{

};

void Fx2_dev::close(){
	delete fx2_conf->d_ephandle6;
	delete fx2_conf->d_devhandle;

	usb_release_interface( fx2_conf->udev, fx2_conf->interface_ );
    usb_reset(fx2_conf->udev);
    usb_close(fx2_conf->udev);
}


int Fx2_dev::read_IF(unsigned char *ch)
{
    unsigned int ui_len =0;
    //fx2_conf->d_devhandle->_reap(false);
    try
    {
        ui_len=fx2_conf->d_ephandle6->read( ch, GN3S_BUFFSIZE);
    }
    catch (const char* e)
    {
        std::cerr<<e<<endl;
    }
    return(ui_len);
}

int Fx2_dev::read_IF_simple(unsigned char *ch, int length)
{
	unsigned int ui_len = 0;
	//fx2_conf->d_devhandle->_reap(false);
	try
	{
		ui_len = fx2_conf->d_ephandle6->read(ch, length);
	}
	catch (const char* e)
	{
		std::cerr << e << endl;
	}
	return(ui_len);
}


int Fx2_dev::read_IF(short *sh)
{
    unsigned int ui_len =0;
    //fx2_conf->d_devhandle->_reap(false);
    try
    {
        ui_len=fx2_conf->d_ephandle6->read( sh, GN3S_BUFFSIZE);
    }
    catch (const char* e)
    {
        std::cerr<<e<<endl;
    }
    return(ui_len);
}


int Fx2_dev::read_AGC(short *agc, bool *RFI,int *RFI_det,unsigned int *agc_count)
{

    unsigned int ret_agc=0;
    unsigned int bufsize_agc = GN3S_AGC_BUFFSIZE;
    bool e_bits =false;
    unsigned char cp_agc_data[64];
    unsigned char uc_flags[5];
    int i_tmp;

    for (i_tmp = 0;i_tmp<64;i_tmp++) //clear buffer
    {
        cp_agc_data[i_tmp]=0;
    }

    // ret_agc = usb_bulk_read(fx2_conf->udev,AGC_ENDPOINT,agc_buf, bufsize_agc, 500);


    //get the AGC data
    usrp_xfer2(VRQ_FLAGS, 0, uc_flags, 5);
    usrp_xfer2(VRQ_GET_AGC, 0, cp_agc_data, 64);

//std::cout<<"recieved agc data "<<(int)cp_agc_data[0]<<" "<<(int)cp_agc_data[1]<<" "<<ret_agc<<endl;
    usrp_xfer (VRQ_AGC, 2);
	i_tmp= 0;
    ret_agc = uc_flags[2];
    if ((int) ret_agc< 0)
    {
        fprintf (stderr, "i:  - usb_bulk_read(EP4): ret_agc = %d (%d) \n",ret_agc,bufsize_agc);
        fprintf (stderr, "%s\n", usb_strerror());
        return(-1);
    }
    else if (ret_agc>=0)
    {
         e_bits =this->agc_parse((char*)cp_agc_data, agc, ret_agc, agc_count, RFI);
    //e_bits =this->agc_parse(agc_buf, agc, ret_agc, agc_count, RFI);
        return(*agc_count);
    }


    return(-1);
}

//Load firmware to the FX2LP
int Fx2_dev::usb_fx2_prepare(const char *firmware_path, void(*preResetCallback)(usb_dev_handle *))
{
	int res = 0;
	usb_dev_handle *udev;

	udev = usb_open(fx2);
	if (!udev)
	{
		fprintf(stderr, "\nCould not obtain a handle to USB GNSS Front-End device \n");
		return -1;
	}

	res = load_firmware(udev, firmware_path, FX_TYPE_FX2LP, preResetCallback);
	return res;
}

int Fx2_dev::usb_fx2_init(uint8_t is_simple_frontend)
{
    char status = 0;
    int interface = RX_INTERFACE;
    int altinterface = RX_ALTINTERFACE;
    fusb_ephandle *d_ephandle6;
    fusb_devhandle *d_devhandle;
    usb_dev_handle *udev;

	if (is_simple_frontend)
	{
		interface = 0;
		altinterface = 1;
	}

    udev = usb_open(fx2);
    if ( !udev )
    {
        fprintf ( stderr, "\nCould not obtain a handle to USB GNSS Front-End device \n" );
        return -1;
    }
    else
    {
        if (usb_set_configuration (udev, 1 ) < 0 )
        {

            fprintf ( stderr,
                      "%s:usb_set_configuration: failed conf %d\n",
                      __FUNCTION__,
                      interface );
            fprintf ( stderr, "%s\n", usb_strerror() );
            usb_close ( udev );
			return -1;
        }

        if ( usb_claim_interface ( udev, interface ) < 0 )
        {
            fprintf ( stderr,
                      "%s:usb_claim_interface: failed interface %d\n",
                      __FUNCTION__,
                      interface );
            fprintf ( stderr, "%s\n", usb_strerror() );
            usb_close ( udev );
			return -1;
        }

        if ( usb_set_altinterface ( udev, altinterface ) < 0 )
        {
            fprintf ( stderr,
                      "%s:usb_set_alt_interface: failed\n",
                      __FUNCTION__ );
            fprintf ( stderr, "%s\n", usb_strerror() );
            usb_release_interface ( udev, interface );
            usb_close( udev );
			return -1;
        }

        d_devhandle=make_devhandle ( udev );

		if (is_simple_frontend)
		{
			d_ephandle6 = d_devhandle->make_ephandle(RX_ENDPOINT,
				true,
				FUSB_SIMPLE_BLOCK_SIZE,
				FUSB_SIMPLE_NBLOCKS);
		}
		else
		{
			d_ephandle6 = d_devhandle->make_ephandle(RX_ENDPOINT,
				true,
				FUSB_BLOCK_SIZE,
				FUSB_NBLOCKS);
		}




        if ( !d_ephandle6->start () )
        {
            fprintf ( stderr, "usrp0_rx: failed to start end point streaming (EP6)" );
            usb_strerror();
            status = -1;
        }

        if (status == 0 )
        {
			fx2_conf->interface_ = interface;
            fx2_conf->altinterface = altinterface;
            fx2_conf->udev = udev;
            fx2_conf->d_devhandle = d_devhandle;
            fx2_conf->d_ephandle6 = d_ephandle6;

            //printf("fx2 initiated\n");
            return 0;
        }
        else
        {
            return -1;
        }
    }
}

// Return 0 if connection fail
int Fx2_dev::usb_fx2_find(void)
{

    struct usb_bus *bus;
    struct usb_device *dev;
    //  struct usb_device *fx2 = NULL;
    usb_dev_handle *udev;
    bool info = false;
    int ret;
    char string[256];
    char *num_str =(char*)("00");
    usb_init();
    //  fx2_port =0;
    usb_find_busses();
    usb_find_devices();


//    printf ( "bus/device idVendor/idProduct, %d pid %d \n",fx2_pid[fx2_port],fx2_vid[fx2_port] );

    for ( bus = usb_busses; bus; bus = bus->next )
    {
        for ( dev = bus->devices; dev; dev = dev->next )
        {
//            std::cout<< fx2_vid[fx2_port] <<" "<<fx2_pid[fx2_port]<<endl;
            if (dev->descriptor.idVendor == fx2_vid[fx2_port] && dev->descriptor.idProduct == fx2_pid[fx2_port])
            {
                //printf("found VID = %d , PID = %d\n" ,dev->descriptor.idVendor, dev->descriptor.idProduct);
                if (strcmp(bus->dirname,num_str )==0)
                {
                    std::cout<<"found device "<<dev->descriptor.idVendor<<" "<< dev->descriptor.idProduct<<endl;
                    fx2= dev;
                    return 1;
                }
                else if (strcmp(num_str,"00")==0)
                {
                    // std::cout<<"did not found device\n";
                    fx2= dev;
                    return 1;
                }
                else
                {
                    //printf
                    return 0;
                }
                std::cout<<"found device 1\n";
                return(0);
            }

            if (info)
            {
                printf("udev/n");
                udev = usb_open ( dev );
                if ( udev )
                {
                    if ( dev->descriptor.iManufacturer )
                    {
                        ret = usb_get_string_simple ( udev,
                                                      dev->descriptor.iManufacturer,
                                                      string,
                                                      sizeof ( string ) );

                        if ( ret > 0 )
                            printf ( "- Manufacturer : %s\n", string );
                        else
                            printf ( "- Unable to fetch manufacturer string\n" );
                    }

                    if ( dev->descriptor.iProduct )
                    {
                        ret = usb_get_string_simple ( udev,
                                                      dev->descriptor.iProduct,
                                                      string,
                                                      sizeof ( string ) );

                        if ( ret > 0 )
                            printf ( "- Product : %s\n", string );
                        else
                            printf ( "- Unable to fetch product string\n" );
                    }

                    if ( dev->descriptor.iSerialNumber )
                    {
                        ret = usb_get_string_simple ( udev,
                                                      dev->descriptor.iSerialNumber,
                                                      string,
                                                      sizeof ( string ) );

                        if ( ret > 0 )
                            printf ( "- Serial Number: %s\n", string );
                        else
                            printf ( "- Unable to fetch serial number string\n" );
                    }
                    usb_close ( udev );
                }

                if ( !dev->config )
                {
                    printf ( " Could not retrieve descriptors\n" );
                    continue;
                }

                for ( int i = 0; i < dev->descriptor.bNumConfigurations; i++ )
                {
                    print_configuration ( &dev->config[i] );
                }
            }
        }
    }
    return 0;
}

void Fx2_dev::print_configuration ( struct usb_config_descriptor *config )
{
    int i;

    printf ( " wTotalLength: %d\n", config->wTotalLength );
    printf ( " bNumInterfaces: %d\n", config->bNumInterfaces );
    printf ( " bConfigurationValue: %d\n", config->bConfigurationValue );
    printf ( " iConfiguration: %d\n", config->iConfiguration );
    printf ( " bmAttributes: %02xh\n", config->bmAttributes );
    printf ( " MaxPower: %d\n", config->MaxPower );

    for ( i = 0; i < config->bNumInterfaces; i++ )
        print_interface ( &config->interface[i] );
}

void Fx2_dev::print_altsetting ( struct usb_interface_descriptor *interface )
{
    int i;

    printf ( " bInterfaceNumber: %d\n", interface->bInterfaceNumber );
    printf ( " bAlternateSetting: %d\n", interface->bAlternateSetting );
    printf ( " bNumEndpoints: %d\n", interface->bNumEndpoints );
    printf ( " bInterfaceClass: %d\n", interface->bInterfaceClass );
    printf ( " bInterfaceSubClass: %d\n", interface->bInterfaceSubClass );
    printf ( " bInterfaceProtocol: %d\n", interface->bInterfaceProtocol );
    printf ( " iInterface: %d\n", interface->iInterface );

    for ( i = 0; i < interface->bNumEndpoints; i++ )
        print_endpoint ( &interface->endpoint[i] );
}

void Fx2_dev::print_interface ( struct usb_interface *interface )
{
    int i;

    for ( i = 0; i < interface->num_altsetting; i++ )
        print_altsetting ( &interface->altsetting[i] );
}

short  Fx2_dev::print_fifo_status (char print )
{
    unsigned char flags[5];
    short idx_data;

    usrp_xfer2 (VRQ_FLAGS, 0, flags, 5 );
    idx_data = flags[4] << 8;
    idx_data |= ( flags[3] & 0xff );

    if ( print )
    {
        printf ( "agc_en: %d (0x%02x) \n", flags[0], flags[0] );
        printf ( "op_mode: %d (0x%02x) \n", flags[1], flags[1] );
        printf ( "ep4_reset: %d (0x%02x) \n", flags[2], flags[2] );
        printf ( "idx: %d \n", idx_data );
        printf ( "\n" );
    }

    return idx_data;
}


void  Fx2_dev::print_endpoint ( struct usb_endpoint_descriptor *endpoint )
{
    printf ( " bEndpointAddress: %02xh\n", endpoint->bEndpointAddress );
    printf ( " bmAttributes: %02xh\n", endpoint->bmAttributes );
    printf ( " wMaxPacketSize: %d\n", endpoint->wMaxPacketSize );
    printf ( " bInterval: %d\n", endpoint->bInterval );
    printf ( " bRefresh: %d\n", endpoint->bRefresh );
    printf ( " bSynchAddress: %d\n", endpoint->bSynchAddress );
}


int Fx2_dev::write_cmd ( int request, int value, int index, unsigned char *bytes, int len )
{
    // int r = write_cmd (udh, VRQ_XFER, start, 0, 0, 0);
    int requesttype = ( request & 0x80 ) ? VRT_VENDOR_IN : VRT_VENDOR_OUT;

    int r = usb_control_msg ( fx2_conf->udev, requesttype, request, value, index,
                              ( char * ) bytes, len, 1000 );
//std::cout<<"tries to write "<<value << " "<< index<< " "<<bytes<<endl;

    if ( r < 0 )
    {
        // we get EPIPE if the firmware stalls the endpoint.
        if ( errno != EPIPE )
            std::cout<<"failed to write "<<value << " "<< index<< " "<<bytes<<endl;
            fprintf ( stderr, "usb_control_msg failed write_cmd_fx2.cpp: %s\n", usb_strerror() );
    }
    return r;

}

bool Fx2_dev::_get_status ( int which, bool *trouble )
{
    unsigned char status;
    *trouble = true;
	if ( write_cmd ( VRQ_GET_STATUS, 0, which, &status, sizeof ( status ) ) != sizeof ( status ) )
        return false;

    *trouble = false;
    return true;
}


bool Fx2_dev::check_rx_overrun ( bool *overrun_p )
{
    return _get_status ( GS_RX_OVERRUN, overrun_p );
}


bool Fx2_dev::usrp_xfer ( unsigned char VRQ_TYPE, unsigned char start )
{
    int r = write_cmd (VRQ_TYPE, start, 0, 0, 0 );
    return r == 0;
}

bool Fx2_dev::usrp_xfer2 ( unsigned char VRQ_TYPE, unsigned char start, unsigned char *buf, char len )
{
    int r = write_cmd (VRQ_TYPE, start, 0, buf, len );
    return r == 0;
}


fusb_devhandle* Fx2_dev::make_devhandle ( usb_dev_handle *udh )
{
    return new fusb_devhandle_win32 ( udh );
}

int Fx2_dev::fx2_usb_bulk_read(char *bytes, int size,int timeout)
{
    return(usb_bulk_read(fx2_conf->udev, AGC_ENDPOINT, bytes,  size, timeout));
};

bool Fx2_dev::agc_parse(char *buf, short *agc, unsigned int agc_size, unsigned int *count, bool *RFI)
{
    float RFI_limit = 1; //Needs to be verified
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int e_headers = 0;
    short buf16 =0;
   // short  header2;//header,
    short data, data2;
    data= 0;
    data2 =0;
    //float agc_f = 1;
    // Parse AGC data
    if (agc_size>0)
    {
        for (i=0; i<agc_size; i+=2)
        {
            buf16 = buf[i+1] << 8;
            buf16 |= (buf[i] & 0xff);
            // header2 = buf16 & 0xf000;
            data2 = buf16 & 0x0fff;
            if ( data2 > 0)  // Break the loop if only zeroes are read
            {
                // if (header2 & bmAGC)
                // {
                // Save AGC data separately
                agc[j] = data2;

                //          header2 = header2 & 0x3000;	// Clear channel bits
                j++;
                //}

            }
        } // end for loop
        *count = j;
    }
    if (e_headers > j/2)
        return true;
    else
        return false;
}


void Fx2_dev::set_port(int i)
{
    fx2_port =i;

}

/*
 * return true if [addr,addr+len] includes external RAM
 * for Cypress EZ-USB FX2LP
 */
static bool fx2lp_is_external(uint32_t addr, size_t len)
{
	/* 1st 16KB for data/code, 0x0000-0x3fff */
	if (addr <= 0x3fff)
		return ((addr + len) > 0x4000);

	/* and 512 for data, 0xe000-0xe1ff */
	else if (addr >= 0xe000 && addr <= 0xe1ff)
		return ((addr + len) > 0xe200);

	/* otherwise, it's certainly external */
	else
		return true;
}

/*
 * return true if [addr,addr+len] includes external RAM
 * for Cypress EZ-USB FX2
 */
static bool fx2_is_external(uint32_t addr, size_t len)
{
	/* 1st 8KB for data/code, 0x0000-0x1fff */
	if (addr <= 0x1fff)
		return ((addr + len) > 0x2000);

	/* and 512 for data, 0xe000-0xe1ff */
	else if (addr >= 0xe000 && addr <= 0xe1ff)
		return ((addr + len) > 0xe200);

	/* otherwise, it's certainly external */
	else
		return true;
}

/*
 * return true if [addr,addr+len] includes external RAM
 * for Anchorchips EZ-USB or Cypress EZ-USB FX
 */
static bool fx_is_external(uint32_t addr, size_t len)
{
	/* with 8KB RAM, 0x0000-0x1b3f can be written
	 * we can't tell if it's a 4KB device here
	 */
	if (addr <= 0x1b3f)
		return ((addr + len) > 0x1b40);

	/* there may be more RAM; unclear if we can write it.
	 * some bulk buffers may be unused, 0x1b3f-0x1f3f
	 * firmware can set ISODISAB for 2KB at 0x2000-0x27ff
	 */
	return true;
}


/*
 * Modifies the CPUCS register to stop or reset the CPU.
 * Returns false on error.
 */
static bool ezusb_cpucs(usb_dev_handle *device, uint32_t addr, bool doRun)
{
	int status;
	char data = doRun ? 0x00 : 0x01;

	status = usb_control_msg(device,
		(LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE),
		RW_INTERNAL, addr & 0xFFFF, addr >> 16,
		&data, 1, 1000);

	if ((status != 1) &&
		/* We may get an I/O error from libusb as the device disappears */
		((!doRun) || (status < 0)))
	{
		const char *mesg = "Can't modify CPUCS";
		if (status < 0)
		{
			fprintf(stderr, "USB ERROR\n");
			//fprintf(stderr, "%s: %s\n", mesg, libusb_error_name(status));
		}
		else
			fprintf(stderr, "%s\n", mesg);
		return false;
	}
	else
		return true;
}



/*
 * Issues the specified vendor-specific write request.
 */
static int ezusb_write(usb_dev_handle *device, const char *label,
	uint8_t opcode, uint32_t addr, const unsigned char *data, size_t len)
{
	int status;

	status = usb_control_msg(device,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		opcode, addr & 0xFFFF, addr >> 16,
		(char*)data, (uint16_t)len, 1000);
	if (status != (signed)len) 
	{
		fprintf(stderr, "USB ERROR\n");
	}
	if (status < 0)
	{
		errno = EIO;
		return -1;
	}
	return 0;
}

int Fx2_dev::ezusb_write2(usb_dev_handle *device, const char *label,
	uint8_t opcode, uint32_t addr, const unsigned char *data, size_t len)
{
	return ezusb_write(device, label, opcode, addr, data, len);
}

static int  parse_ihex(FILE *image, void *context,
	bool(*is_external)(uint32_t addr, size_t len),
	int(*poke) (void *context, uint32_t addr, bool external,
		const unsigned char *data, size_t len))
{
	unsigned char data[1023];
	uint32_t data_addr = 0;
	size_t data_len = 0;
	int rc;
	int first_line = 1;
	bool external = false;

	/* Read the input file as an IHEX file, and report the memory segments
	 * as we go.  Each line holds a max of 16 bytes, but uploading is
	 * faster (and EEPROM space smaller) if we merge those lines into larger
	 * chunks.  Most hex files keep memory segments together, which makes
	 * such merging all but free.  (But it may still be worth sorting the
	 * hex files to make up for undesirable behavior from tools.)
	 *
	 * Note that EEPROM segments max out at 1023 bytes; the upload protocol
	 * allows segments of up to 64 KBytes (more than a loader could handle).
	 */
	for (;;) 
	{
		char buf[512], *cp;
		char tmp, type;
		size_t len;
		unsigned idx, off;

		cp = fgets(buf, sizeof(buf), image);
		if (cp == NULL) {
			fprintf(stderr, "EOF without EOF record!\n");
			break;
		}

		/* EXTENSION: "# comment-till-end-of-line", for copyrights etc */
		if (buf[0] == '#')
			continue;

		if (buf[0] != ':') {
			fprintf(stderr, "Not an ihex record: %s", buf);
			return -2;
		}

		/* ignore any newline */
		cp = strchr(buf, '\n');
		if (cp)
			*cp = 0;

		/* Read the length field (up to 16 bytes) */
		tmp = buf[3];
		buf[3] = 0;
		len = strtoul(buf + 1, NULL, 16);
		buf[3] = tmp;

		/* Read the target offset (address up to 64KB) */
		tmp = buf[7];
		buf[7] = 0;
		off = (unsigned int)strtoul(buf + 3, NULL, 16);
		buf[7] = tmp;

		/* Initialize data_addr */
		if (first_line) {
			data_addr = off;
			first_line = 0;
		}

		/* Read the record type */
		tmp = buf[9];
		buf[9] = 0;
		type = (char)strtoul(buf + 7, NULL, 16);
		buf[9] = tmp;

		/* If this is an EOF record, then make it so. */
		if (type == 1) 
		{
			break;
		}

		if (type != 0) {
			fprintf(stderr, "Unsupported record type: %u\n", type);
			return -3;
		}

		if ((len * 2) + 11 > strlen(buf)) {
			fprintf(stderr, "Record too short?\n");
			return -4;
		}

		/* FIXME check for _physically_ contiguous not just virtually
		 * e.g. on FX2 0x1f00-0x2100 includes both on-chip and external
		 * memory so it's not really contiguous */

		 /* flush the saved data if it's not contiguous,
		 * or when we've buffered as much as we can.
		 */
		if (data_len != 0
			&& (off != (data_addr + data_len)
				/* || !merge */
				|| (data_len + len) > sizeof(data))) {
			if (is_external)
				external = is_external(data_addr, data_len);
			rc = poke(context, data_addr, external, data, data_len);
			if (rc < 0)
				return -1;
			data_addr = off;
			data_len = 0;
		}

		/* append to saved data, flush later */
		for (idx = 0, cp = buf + 9; idx < len; idx += 1, cp += 2) {
			tmp = cp[2];
			cp[2] = 0;
			data[data_len + idx] = (uint8_t)strtoul(cp, NULL, 16);
			cp[2] = tmp;
		}
		data_len += len;
	}//FOR


	/* flush any data remaining */
	if (data_len != 0) 
	{
		if (is_external)
			external = is_external(data_addr, data_len);
		rc = poke(context, data_addr, external, data, data_len);
		if (rc < 0)
			return -1;
	}
	return 0;
}

static int ram_poke(void *context, uint32_t addr, bool external,
	const unsigned char *data, size_t len)
{
	struct ram_poke_context *ctx = (struct ram_poke_context*)context;
	int rc;
	unsigned retry = 0;

	switch (ctx->mode) 
	{
	case internal_only:		/* CPU should be stopped */
		if (external) {
			fprintf(stderr, "Can't write %u bytes external memory at 0x%08x\n",
				(unsigned)len, addr);
			errno = EINVAL;
			return -1;
		}
		break;
	case skip_internal:		/* CPU must be running */
		fprintf(stderr, "Not implemented!\n");
		break;
	case skip_external:		/* CPU should be stopped */
		fprintf(stderr, "Not implemented!\n");
		break;
	case _undef:
	default:
		fprintf(stderr, "Bug!\n");
		errno = EDOM;
		return -1;
	}

	ctx->total += len;
	ctx->count++;

	/* Retry this till we get a real error. Control messages are not
	 * NAKed (just dropped) so time out means is a real problem.
	 */
	while ((rc = ezusb_write(ctx->device,
		external ? "Write external" : "Write on-chip",
		external ? RW_MEMORY : RW_INTERNAL,
		addr, data, len)) < 0
		&& retry < RETRY_LIMIT)
	{
		if (rc != 0)
			break;
		retry += 1;
	}
	return rc;
}

/*
 * Load a firmware file into target RAM. device is the open libusb
 * device, and the path is the name of the source file. Open the file,
 * parse the bytes, and write them in one or two phases.
 *
 * If stage == 0, this uses the first stage loader, built into EZ-USB
 * hardware but limited to writing on-chip memory or CPUCS.  Everything
 * is written during one stage, unless there's an error such as the image
 * holding data that needs to be written to external memory.
 *
 * Otherwise, things are written in two stages.  First the external
 * memory is written, expecting a second stage loader to have already
 * been loaded.  Then file is re-parsed and on-chip memory is written.
 */
int Fx2_dev::load_firmware(usb_dev_handle *device, const char *path, int fx_type,
	void(*preResetCallback)(usb_dev_handle *))
{
	
	FILE *image;
	uint32_t cpucs_addr;
	bool(*is_external)(uint32_t off, size_t len);
	struct ram_poke_context ctx;
	int status;
	uint8_t iic_header[8] = { 0 };
	int ret = 0;

	image = fopen(path, "rb");
	if (image == NULL) 
	{
		fprintf(stderr, "%s: unable to open for input.\n", path);
		return -2;
	}
	else
	{
		fprintf(stderr, "Open firmware image %s for RAM upload\n", path);
	}

	// EZ-USB original/FX and FX2 devices differ, apart from the 8051 core 
	switch (fx_type) 
	{
	case FX_TYPE_FX2LP:
		cpucs_addr = 0xe600;
		is_external = fx2lp_is_external;
		break;
	case FX_TYPE_FX2:
		cpucs_addr = 0xe600;
		is_external = fx2_is_external;
		break;
	default:
		cpucs_addr = 0x7f92;
		is_external = fx_is_external;
		break;
	}
	
	ctx.mode = internal_only;
	// if required, halt the CPU while we overwrite its code/data 
	if (cpucs_addr && !ezusb_cpucs(device, cpucs_addr, false))
	{
		ret = -1;
		goto exit;
	}

	// scan the image, first (maybe only) time 
	ctx.device = device;
	ctx.total = ctx.count = 0;
	status = parse_ihex(image, &ctx, is_external, ram_poke);
	if (status < 0) {
		fprintf(stderr, "Unable to upload %s\n", path);
		ret = status;
		goto exit;
	}
	
	if ((ctx.count != 0)) {
		fprintf(stderr, "... Wrote: %d bytes, %d segments, avg %d\n",
			(int)ctx.total, (int)ctx.count, (int)(ctx.total / ctx.count));
	}

	if (preResetCallback != NULL) 
		preResetCallback(device);

	// if required, reset the CPU so it runs what we just uploaded 
	if (cpucs_addr && !ezusb_cpucs(device, cpucs_addr, true))
		ret = -1;
	
	usb_release_interface(device, 0);
	usb_close(device);
exit:
	fclose(image);
	return ret;

return -1;
}

