#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
	FILE *fileptr;
	unsigned char *buffer;
	long filelen;
	int i = 0;

	fileptr = fopen(argv[1], "rb");     
	if (fileptr == NULL)
	{
		fprintf(stderr, "cannot open input file\n");
		return 1;
    }

	fseek(fileptr, 0, SEEK_END);          
	filelen = ftell(fileptr);            
	rewind(fileptr);                      
	buffer = (unsigned char *)malloc((filelen+1)*sizeof(unsigned char)); 

	fread(buffer, filelen, 1, fileptr);

	fclose(fileptr);

	fileptr = fopen(argv[2], "w");
	if (fileptr == NULL)
	{
		fprintf(stderr, "cannot open ouput file\n");
		return 1;
    }
	
	fprintf(fileptr, "static const uint8_t SHD_FIRMWARE_MAJOR_VERSION = %d;\n", atoi(argv[3]));
	fprintf(fileptr, "static const uint8_t SHD_FIRMWARE_MINOR_VERSION = %d;\n", atoi(argv[4]));
	fprintf(fileptr, "const uint8_t stm_firmware[] PROGMEM = {\n");
	while (i < filelen)
	{
		fprintf(fileptr, "    ");
		for (int j = 0; j < 12; j++)
			fprintf(fileptr, "0x%.2x, ", buffer[i++]);
		fprintf(fileptr, "\n");
	}
	fprintf(fileptr, "};\n\n");

	return 0;
}