/*
	CRC-16 0x8005

	This function is called to generate the Cyclic Redundancy Check
	code using the CRC16 algorithm.  The table lookup method
	is used for execution speed.  The CRC code is used to detect
	data errors on messages.

	See http://www.lammertbies.nl/comm/info/crc-calculation.html
*/
#include "crc-16.h"

unsigned short int CRC16( char *Buffer, int Size )
{
/*
	Routine to generate the CRC-16 code for a buffer of length size
	Crc16(*Buffer, Size)
	Buffer		input data buffer
	Size		length in bytes of buffer
*/
static unsigned short CRCtbl[256] = {
0x0000,0xc0c1,0xc181,0x0140,0xc301,0x03c0,0x0280,0xc241,
0xc601,0x06c0,0x0780,0xc741,0x0500,0xc5c1,0xc481,0x0440,
0xcc01,0x0cc0,0x0d80,0xcd41,0x0f00,0xcfc1,0xce81,0x0e40,
0x0a00,0xcac1,0xcb81,0x0b40,0xc901,0x09c0,0x0880,0xc841,
0xd801,0x18c0,0x1980,0xd941,0x1b00,0xdbc1,0xda81,0x1a40,
0x1e00,0xdec1,0xdf81,0x1f40,0xdd01,0x1dc0,0x1c80,0xdc41,
0x1400,0xd4c1,0xd581,0x1540,0xd701,0x17c0,0x1680,0xd641,
0xd201,0x12c0,0x1380,0xd341,0x1100,0xd1c1,0xd081,0x1040,
0xf001,0x30c0,0x3180,0xf141,0x3300,0xf3c1,0xf281,0x3240,
0x3600,0xf6c1,0xf781,0x3740,0xf501,0x35c0,0x3480,0xf441,
0x3c00,0xfcc1,0xfd81,0x3d40,0xff01,0x3fc0,0x3e80,0xfe41,
0xfa01,0x3ac0,0x3b80,0xfb41,0x3900,0xf9c1,0xf881,0x3840,
0x2800,0xe8c1,0xe981,0x2940,0xeb01,0x2bc0,0x2a80,0xea41,
0xee01,0x2ec0,0x2f80,0xef41,0x2d00,0xedc1,0xec81,0x2c40,
0xe401,0x24c0,0x2580,0xe541,0x2700,0xe7c1,0xe681,0x2640,
0x2200,0xe2c1,0xe381,0x2340,0xe101,0x21c0,0x2080,0xe041,
0xa001,0x60c0,0x6180,0xa141,0x6300,0xa3c1,0xa281,0x6240,
0x6600,0xa6c1,0xa781,0x6740,0xa501,0x65c0,0x6480,0xa441,
0x6c00,0xacc1,0xad81,0x6d40,0xaf01,0x6fc0,0x6e80,0xae41,
0xaa01,0x6ac0,0x6b80,0xab41,0x6900,0xa9c1,0xa881,0x6840,
0x7800,0xb8c1,0xb981,0x7940,0xbb01,0x7bc0,0x7a80,0xba41,
0xbe01,0x7ec0,0x7f80,0xbf41,0x7d00,0xbdc1,0xbc81,0x7c40,
0xb401,0x74c0,0x7580,0xb541,0x7700,0xb7c1,0xb681,0x7640,
0x7200,0xb2c1,0xb381,0x7340,0xb101,0x71c0,0x7080,0xb041,
0x5000,0x90c1,0x9181,0x5140,0x9301,0x53c0,0x5280,0x9241,
0x9601,0x56c0,0x5780,0x9741,0x5500,0x95c1,0x9481,0x5440,
0x9c01,0x5cc0,0x5d80,0x9d41,0x5f00,0x9fc1,0x9e81,0x5e40,
0x5a00,0x9ac1,0x9b81,0x5b40,0x9901,0x59c0,0x5880,0x9841,
0x8801,0x48c0,0x4980,0x8941,0x4b00,0x8bc1,0x8a81,0x4a40,
0x4e00,0x8ec1,0x8f81,0x4f40,0x8d01,0x4dc0,0x4c80,0x8c41,
0x4400,0x84c1,0x8581,0x4540,0x8701,0x47c0,0x4680,0x8641,
0x8201,0x42c0,0x4380,0x8341,0x4100,0x81c1,0x8081,0x4040
};
	unsigned short CRC;
	unsigned char C;

	for ( CRC=0; Size ; Size--)
	{
	   C = *Buffer++ ^ (char)CRC;
	   CRC >>= 8;
	   CRC ^= CRCtbl[C];
	}
	return( CRC );
}

// THE AWK CRC-16 PROGRAM THAT MATCHES THIS CRC-16
//
//BEGIN{
//# # Initialize CRC-16 0x8005 table
//
//T[0]=0x0000;T[1]=0xc0c1;T[2]=0xc181;T[3]=0x0140;T[4]=0xc301;T[5]=0x03c0;T[6]=0x0280;T[7]=0xc241;
//T[8]=0xc601;T[9]=0x06c0;T[10]=0x0780;T[11]=0xc741;T[12]=0x0500;T[13]=0xc5c1;T[14]=0xc481;T[15]=0x0440;
//T[16]=0xcc01;T[17]=0x0cc0;T[18]=0x0d80;T[19]=0xcd41;T[20]=0x0f00;T[21]=0xcfc1;T[22]=0xce81;T[23]=0x0e40;
//T[24]=0x0a00;T[25]=0xcac1;T[26]=0xcb81;T[27]=0x0b40;T[28]=0xc901;T[29]=0x09c0;T[30]=0x0880;T[31]=0xc841;
//T[32]=0xd801;T[33]=0x18c0;T[34]=0x1980;T[35]=0xd941;T[36]=0x1b00;T[37]=0xdbc1;T[38]=0xda81;T[39]=0x1a40;
//T[40]=0x1e00;T[41]=0xdec1;T[42]=0xdf81;T[43]=0x1f40;T[44]=0xdd01;T[45]=0x1dc0;T[46]=0x1c80;T[47]=0xdc41;
//T[48]=0x1400;T[49]=0xd4c1;T[50]=0xd581;T[51]=0x1540;T[52]=0xd701;T[53]=0x17c0;T[54]=0x1680;T[55]=0xd641;
//T[56]=0xd201;T[57]=0x12c0;T[58]=0x1380;T[59]=0xd341;T[60]=0x1100;T[61]=0xd1c1;T[62]=0xd081;T[63]=0x1040;
//T[64]=0xf001;T[65]=0x30c0;T[66]=0x3180;T[67]=0xf141;T[68]=0x3300;T[69]=0xf3c1;T[70]=0xf281;T[71]=0x3240;
//T[72]=0x3600;T[73]=0xf6c1;T[74]=0xf781;T[75]=0x3740;T[76]=0xf501;T[77]=0x35c0;T[78]=0x3480;T[79]=0xf441;
//T[80]=0x3c00;T[81]=0xfcc1;T[82]=0xfd81;T[83]=0x3d40;T[84]=0xff01;T[85]=0x3fc0;T[86]=0x3e80;T[87]=0xfe41;
//T[88]=0xfa01;T[89]=0x3ac0;T[90]=0x3b80;T[91]=0xfb41;T[92]=0x3900;T[93]=0xf9c1;T[94]=0xf881;T[95]=0x3840;
//T[96]=0x2800;T[97]=0xe8c1;T[98]=0xe981;T[99]=0x2940;T[100]=0xeb01;T[101]=0x2bc0;T[102]=0x2a80;T[103]=0xea41;
//T[104]=0xee01;T[105]=0x2ec0;T[106]=0x2f80;T[107]=0xef41;T[108]=0x2d00;T[109]=0xedc1;T[110]=0xec81;T[111]=0x2c40;
//T[112]=0xe401;T[113]=0x24c0;T[114]=0x2580;T[115]=0xe541;T[116]=0x2700;T[117]=0xe7c1;T[118]=0xe681;T[119]=0x2640;
//T[120]=0x2200;T[121]=0xe2c1;T[122]=0xe381;T[123]=0x2340;T[124]=0xe101;T[125]=0x21c0;T[126]=0x2080;T[127]=0xe041;
//T[128]=0xa001;T[129]=0x60c0;T[130]=0x6180;T[131]=0xa141;T[132]=0x6300;T[133]=0xa3c1;T[134]=0xa281;T[135]=0x6240;
//T[136]=0x6600;T[137]=0xa6c1;T[138]=0xa781;T[139]=0x6740;T[140]=0xa501;T[141]=0x65c0;T[142]=0x6480;T[143]=0xa441;
//T[144]=0x6c00;T[145]=0xacc1;T[146]=0xad81;T[147]=0x6d40;T[148]=0xaf01;T[149]=0x6fc0;T[150]=0x6e80;T[151]=0xae41;
//T[152]=0xaa01;T[153]=0x6ac0;T[154]=0x6b80;T[155]=0xab41;T[156]=0x6900;T[157]=0xa9c1;T[158]=0xa881;T[159]=0x6840;
//T[160]=0x7800;T[161]=0xb8c1;T[162]=0xb981;T[163]=0x7940;T[164]=0xbb01;T[165]=0x7bc0;T[166]=0x7a80;T[167]=0xba41;
//T[168]=0xbe01;T[169]=0x7ec0;T[170]=0x7f80;T[171]=0xbf41;T[172]=0x7d00;T[173]=0xbdc1;T[174]=0xbc81;T[175]=0x7c40;
//T[176]=0xb401;T[177]=0x74c0;T[178]=0x7580;T[179]=0xb541;T[180]=0x7700;T[181]=0xb7c1;T[182]=0xb681;T[183]=0x7640;
//T[184]=0x7200;T[185]=0xb2c1;T[186]=0xb381;T[187]=0x7340;T[188]=0xb101;T[189]=0x71c0;T[190]=0x7080;T[191]=0xb041;
//T[192]=0x5000;T[193]=0x90c1;T[194]=0x9181;T[195]=0x5140;T[196]=0x9301;T[197]=0x53c0;T[198]=0x5280;T[199]=0x9241;
//T[200]=0x9601;T[201]=0x56c0;T[202]=0x5780;T[203]=0x9741;T[204]=0x5500;T[205]=0x95c1;T[206]=0x9481;T[207]=0x5440;
//T[208]=0x9c01;T[209]=0x5cc0;T[210]=0x5d80;T[211]=0x9d41;T[212]=0x5f00;T[213]=0x9fc1;T[214]=0x9e81;T[215]=0x5e40;
//T[216]=0x5a00;T[217]=0x9ac1;T[218]=0x9b81;T[219]=0x5b40;T[220]=0x9901;T[221]=0x59c0;T[222]=0x5880;T[223]=0x9841;
//T[224]=0x8801;T[225]=0x48c0;T[226]=0x4980;T[227]=0x8941;T[228]=0x4b00;T[229]=0x8bc1;T[230]=0x8a81;T[231]=0x4a40;
//T[232]=0x4e00;T[233]=0x8ec1;T[234]=0x8f81;T[235]=0x4f40;T[236]=0x8d01;T[237]=0x4dc0;T[238]=0x4c80;T[239]=0x8c41;
//T[240]=0x4400;T[241]=0x84c1;T[242]=0x8581;T[243]=0x4540;T[244]=0x8701;T[245]=0x47c0;T[246]=0x4680;T[247]=0x8641;
//T[248]=0x8201;T[249]=0x42c0;T[250]=0x4380;T[251]=0x8341;T[252]=0x4100;T[253]=0x81c1;T[254]=0x8081;T[255]=0x4040;
//
//# Init raw data to int lookup table
//  for(i=0;i<=255;i++) {X[sprintf("%c",i)]=i;}
//
//}  # Close the BEGIN
//
//function CRC16(buf)
//{
//  CRC=0; #// Initial seed.
//
//  # We assume "buf" contains data.
//
//  A[0]=split(buf,A,"");       #A[0] has the length, A[...] has each character to use.
//
//  for(i=1;i<=A[0]; i++)
//	{
//	C = and(xor(CRC, X[A[i]] ), 0x00FF);
//	CRC = rshift(CRC,8);
//	CRC = xor(CRC, T[C]);
//	}
//
//  return CRC;
//}
//# See http://www.lammertbies.nl/comm/info/crc-calculation.html
