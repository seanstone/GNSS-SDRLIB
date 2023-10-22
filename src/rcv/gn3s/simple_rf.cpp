/*------------------------------------------------------------------------------
* simple_rf.cpp : Simpe front end functions
*
* Copyright (C) 2023 iliasam
*-----------------------------------------------------------------------------*/
#include "sdr.h"
#include "fx2.h"

/* global variables ----------------------------------------------------------*/
using namespace std;
Fx2_dev fx2_d2;

uint32_t simple_rf_read_buf_size = 0;
const int FirmwareConfigAddr = 0x1003;

static void pre_reset_callback(usb_dev_handle *device)
{
	uint8_t firmware_config[6] = {18, 67, 224, 12, 16, 0};
	
	int result = fx2_d2.ezusb_write2(
		device, "Write config", 0xA0, FirmwareConfigAddr, firmware_config, ARRAYSIZE(firmware_config));
	if (result < 0) 
	{
		SDRPRINTF("Error writing config in USB controller.\n");
	}
	
}

/* GN3S initialization ---------------------------------------------------------
* search front end and initialization
* args   : none
* return : int status 0:okay -1:failure
*-----------------------------------------------------------------------------*/
extern int simple_rf_init(void) 
{
    unsigned char uc_flags[5];

	char firmware_path[500] = {0};
	//char *firmware_path = ".\\frontend\\fx2pipe.ihx";

#ifdef WIN32
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];

	GetModuleFileNameA(NULL, firmware_path, MAX_PATH);
	string orig_path_str = firmware_path;
	errno_t err = _splitpath_s(orig_path_str.c_str(), drive, _MAX_DRIVE, dir, _MAX_DIR, fname, _MAX_FNAME, ext, _MAX_EXT);
	std::string path_str(std::string(drive) + std::string(dir));
	strcpy(firmware_path, path_str.c_str());
	strcat(firmware_path, "frontend\\fx2pipe.ihx");
#endif

    /* Look for a device */
    fx2_d2 = Fx2_dev(4); /* VID_04B4&PID_8613 */
    if (fx2_d2.usb_fx2_find() == 0) 
	{
		SDRPRINTF("error: no Simple RF frontend found\n");
		return -1;
    } 

	//Load fx2pipe firmware to the FX2
	int prepare_res = fx2_d2.usb_fx2_prepare(firmware_path, pre_reset_callback);
	if (prepare_res < 0)
	{
		SDRPRINTF("error: Can't load firmware to the frontend\n");
		return -1;
	}

	fx2_d2.usb_fx2_init(1);
    return 0;
}



/* stop front-end --------------------------------------------------------------
* stop grabber of front end
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern void simple_rf_quit(void) 
{
    fx2_d2.close();
    fx2_d2.~Fx2_dev();
}


/* data expansion to binary (Simple frontend) ------------------------------------------
* get current data buffer from memory buffer
* args   : char   *buf      I   memory buffer
*          int    n         I   number of grab data
*          int    i_mode    I   GN3S data grabber mode
*          char   *expbuff  O   extracted data buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void simple_rf_exp(unsigned char *buf, int n, char *expbuf)
{
	memcpy(expbuf, buf, n);
}
/* get current data buffer (Simple frontend) -------------------------------------------
* get current data buffer from local memory buffer
* args   : uint64_t buffloc I   buffer location
*          int    n         I   number of grab data
*          int    dtype     I   data type (DTYPEI or DTYPEIQ)
*          char   *expbuff  O   extracted data buffer - destination
* return : none
*-----------------------------------------------------------------------------*/
extern void simple_rf_getbuf(uint64_t buffloc, int n, int dtype, char *expbuf)
{
    uint64_t membuffloc = buffloc % (MEMBUFFLEN * simple_rf_read_buf_size);
    int nout = (int)((membuffloc + n) - (MEMBUFFLEN * simple_rf_read_buf_size));

    mlock(hbuffmtx);
	if (nout > 0)
	{
		memcpy(expbuf, &sdrstat.buff[membuffloc], n - nout);
		memcpy(&expbuf[n - nout], &sdrstat.buff[0], nout);
	}
	else
	{
		memcpy(expbuf, &sdrstat.buff[membuffloc], n);
	}
    unmlock(hbuffmtx);
}

//Make four 2bit samples from one 
void simple_rf_convert_8bit(uint8_t *src_buf, uint8_t *dst_buf)
{
	int i = 0;
	int i_idx = 0;
	int sample = 0;
	int src_length = SIMPLE_RF_BUFFSIZE / 4;

	char LUT_2bit[4]={+1, +3, -1, -3};
	//char LUT_2bit[4] = { 0x1, 0x3, 0xFF, 0xFD };

	uint8_t res_byte;
	uint8_t byteValUp;
	uint8_t byteValLow;
	uint8_t src_byte;

	for (i = 0; i < src_length; i++)
	{
		src_byte = src_buf[i];
		for (sample = 0; sample < 4; sample++)
		{
			res_byte = 0;
			if ((src_byte & 8) != 0)
				res_byte |= 1; //mag
			if ((src_byte & 128) != 0)
				res_byte |= 2; //sign

			dst_buf[i_idx] = LUT_2bit[res_byte];
			src_byte = src_byte << 1;
			i_idx++;
		}
	}
}

UINT64 simple_rf_test_rx_cnt = 0;
int simple_rf_test_marker_err_cnt = 0;

uint32_t simple_rf_check_markers(uint8_t *data, int length)
{
	static uint8_t state_cnt = 0;
	static uint8_t expected_byte = 0xAA;
	const uint8_t expected_table[4] = { 0xAA, 0xBB, 0xCC, 0xDD };
	static UINT64 simple_rf_prev_marker = 0;
	static uint8_t prev_packet_id = 0;

	uint32_t total_diff = 0;


	uint8_t curr_byte;
	for (int i = 0; i < length; i++)
	{
		curr_byte = data[i];
		simple_rf_test_rx_cnt++;

		if (curr_byte == expected_byte)
		{
			state_cnt++;
			expected_byte = expected_table[state_cnt];
			uint8_t packet_id = data[i + 1];
			if (state_cnt >= 4)
			{
				state_cnt = 0;
				expected_byte = 0xAA;
				UINT64 length_diff = simple_rf_test_rx_cnt - simple_rf_prev_marker;
				simple_rf_prev_marker = simple_rf_test_rx_cnt;
				if (length_diff != 10000)
				{
					simple_rf_test_marker_err_cnt++;
					char textBuf[100];
					sprintf(textBuf, "ERROR: %llu | %d | %d", length_diff, prev_packet_id, packet_id);
					String^ clistr = gcnew String(textBuf);
					System::Diagnostics::Debug::WriteLine(clistr);
					//System::Diagnostics::Debug::WriteLine("MARKER ERROR\n");

					uint8_t diff_packet = packet_id - prev_packet_id;
					if (diff_packet != 1)
					{
						System::Diagnostics::Debug::WriteLine(">>>BIG ERR");
					}

					if ((length_diff < 6000) || (length_diff > 13000))
					{
						UINT64 countM = simple_rf_test_rx_cnt / (1024 * 1024);
						sprintf(textBuf, "COUNT: %llu\n", countM);
						clistr = gcnew String(textBuf);
						System::Diagnostics::Debug::WriteLine(clistr);
					}

					if ((length_diff < 10000) && (diff_packet == 1))
						total_diff += (10000 - length_diff);

					//MessageBeep(MB_ICONEXCLAMATION);
				}
				prev_packet_id = packet_id;
			}
		}
		else
		{
			if (curr_byte == 0xAA)
			{
				state_cnt = 1;
				expected_byte = 0xBB;
			}
			else
			{
				state_cnt = 0;
				expected_byte = 0xAA;
			}
		}
	}
	return total_diff;
}



//Called from rcvgrabdata(), which is called in startsdr() thread
/* push data to memory buffer --------------------------------------------------
* copy data to internal local buffer from front end buffer
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern int simple_rf_pushtomembuf(void)
{
	bool b_overrun = false;
	static uint32_t prev_diff_bytes = 0;

	//One USB byte is 4 ADC samples
	uint8_t tmp_usb_buf[SIMPLE_RF_BUFFSIZE / 4 ];
	uint8_t *read_ptr = tmp_usb_buf;
	uint8_t extracted_buf[SIMPLE_RF_BUFFSIZE];


	int bytes_to_read = SIMPLE_RF_BUFFSIZE / 4;
	if (prev_diff_bytes > 0)
	{
		bytes_to_read -= prev_diff_bytes;
		read_ptr += prev_diff_bytes;
	}
	int read_bytes = fx2_d2.read_IF_simple(read_ptr, bytes_to_read);

	
	uint32_t diff_bytes = simple_rf_check_markers(tmp_usb_buf, SIMPLE_RF_BUFFSIZE / 4);
	prev_diff_bytes = diff_bytes;

	if (read_bytes == bytes_to_read)
	{
		simple_rf_convert_8bit(tmp_usb_buf, extracted_buf);
		mlock(hbuffmtx);
		uint8_t *dst_p = &sdrstat.buff[(sdrstat.buffcnt % MEMBUFFLEN) * simple_rf_read_buf_size];
		memcpy(dst_p, extracted_buf, SIMPLE_RF_BUFFSIZE);
		unmlock(hbuffmtx);
	}
    
    if (read_bytes != bytes_to_read)
	{
        SDRPRINTF("Simple frontend read IF error...\n");
    }

    mlock(hreadmtx);
    sdrstat.buffcnt++;
    unmlock(hreadmtx);

    return 0;
}
/* push data to memory buffer --------------------------------------------------
* post-processing function: push data to memory buffer from binary IF FILE
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
extern void simple_rf_file_pushtomembuf(void) 
{
    size_t nread;

    mlock(hbuffmtx);

    nread=fread(&sdrstat.buff[(sdrstat.buffcnt%MEMBUFFLEN)*simple_rf_read_buf_size],
        1, simple_rf_read_buf_size,sdrini.fp1);

    unmlock(hbuffmtx);

    if (nread< simple_rf_read_buf_size) 
	{
        sdrstat.stopflag=ON;
        SDRPRINTF("end of file!\n");
    }

    mlock(hreadmtx);
    sdrstat.buffcnt++;
    unmlock(hreadmtx);
}
/* get current data buffer from IF FILE ----------------------------------------
* post-processing function: get current data buffer from memory buffer
* args   : uint64_t buffloc I   buffer location
*          int    n         I   number of grab data 
*          int    dtype     I   data type (DTYPEI or DTYPEIQ)
*          char   *expbuff  O   extracted data buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void simple_rf_getbuf_file(uint64_t buffloc, int n, int dtype, char *expbuf)
{
    uint64_t membuffloc=dtype*buffloc%(MEMBUFFLEN*dtype*simple_rf_read_buf_size);
    int nout;

    n=dtype * n;
    nout=(int)((membuffloc+n)-(MEMBUFFLEN*dtype*simple_rf_read_buf_size));

    mlock(hbuffmtx);
    if (nout>0) {
        memcpy(expbuf,&sdrstat.buff[membuffloc],n-nout);
        memcpy(&expbuf[(n-nout)],&sdrstat.buff[0],nout);
    } else {
        memcpy(expbuf,&sdrstat.buff[membuffloc],n);
    }
    unmlock(hbuffmtx);
}

//Expected, that we get "buf_size" samples per one transfer
extern void simple_rf_set_rx_buf(uint32_t buf_size)
{
	simple_rf_read_buf_size = buf_size;
}
