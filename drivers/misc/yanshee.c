//By tango for YANSHEE LOGO from BMP TO RGB565
//use gcc yanshee.c -o yanshee ; ./yanshee $NAME
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include<unistd.h>
#include <strings.h>
#include <stdlib.h>
#include <stdbool.h>
#define LOGO_BIN "logo.bin"

#define WIDTH 320
#define HEIGHT 240

typedef unsigned char uint8;
typedef char int8;
typedef unsigned short  uint16; //32bit system
typedef short  int16;
typedef unsigned int uint32;//32bit system
typedef int int32;


struct bmp_filehead //14B
{
    //2B 'BM'=0x424d;   
    uint16 bmtype; //'BM',2B  
    uint32 bmSize; //size,4B  
    uint32 bmReserved;//0x00,4B
    uint32 bmOffBits;//4B,... 54

}head1;

struct bmphead //40B
{
    uint32 bmpOffBits;//4B,... 40
    uint32 bmp_wide;//4B,wide
    uint32 bmp_high;//4B,high 
    uint16 bmplans;//2B,0 or 1
    uint16 bitcount;//2B ,24
    uint32 bmpyasuo;//4B,0
    uint32 imagesize;//4B,w*h*3

    uint32 biXPelsPerMeter;
    uint32 biYPelsPerMeter;
    uint32 biClrUsed;
    uint32 biClrImportant;
}head2;

struct BGR_data //40B
{
    uint8 B_data;
    uint8 G_data;
    uint8 R_data;
}clr_data[WIDTH*HEIGHT*3]; //img size must less than 1024*768 

int main(int argc, char *argv[])
{
    unsigned char *image_data = (unsigned char *)malloc(WIDTH*HEIGHT*2);
    unsigned int image_index = 0;
    FILE *fp,*out_fp;
    int fd;
    int32 k,q;
    uint32 rec_cout,rec_cout2;
    uint16 tp_16bit;
    int8 src_file[64] = "";
    int8 dst_file[64] = "";

	char buf[32];

    	if(argc < 2)
    	{
        	printf("\nUsage:pls input src file path and dst file path\n");
        	return -1;
    	}
    	else
    	{
    		printf("please input 24 bit(WIDTH*HEIGHT) BMP file\n");
    	}
    
	//strcpy(dst_file, LOGO_BIN);
        //printf("dst file name : ");

	if ((fd = open("st7789v_initlogo.h", O_WRONLY | O_CREAT ,0666)) < 1)
        	return false;

	sprintf(buf,"unsigned char initlogo[]={");
	write(fd,buf,strlen(buf));
	
	for(int i =1; i < argc; i++)
	{
		strcpy(src_file, argv[i]);
    		if((fp=fopen(src_file,"r"))==NULL)
        		printf("open error!");
    		else
    		{
        		fseek(fp, 0, 0);//fseek(fp, offset, fromwhere);
        		rec_cout=fread(&head1,sizeof(head1),1,fp);

        		fseek(fp, 14, 0);//fseek(fp, offset, fromwhere);
        		rec_cout2=fread(&head2,sizeof(head2),1,fp);

        		fseek(fp, 54, 0);//fseek(fp, offset, fromwhere);
        		rec_cout2=fread(&clr_data,3,(head2.bmp_wide*head2.bmp_high),fp);
			printf("with = %d high = %d\n",head2.bmp_wide,head2.bmp_high);

        		fclose(fp);

        		printf("open ok!");
        		if(rec_cout>=0)
        		{
            			printf("write bmp 565 start !\n");
            			for(k=0;k<head2.bmp_high;k++)
                			for(q=0;q<head2.bmp_wide;q++)
                			{
                    				tp_16bit=((uint16)(clr_data[k*head2.bmp_wide+q].R_data>>3)<<11)//R G B
                        				+((uint16)(clr_data[k*head2.bmp_wide+q].G_data>>2)<<5)
                        				+(uint16)(clr_data[k*head2.bmp_wide+q].B_data>>3);
                    				image_data[image_index++] = (uint8)tp_16bit&0xff;
                    				image_data[image_index++] = (uint8)tp_16bit>>8;
						//if( head2.bmp_high-1 == k && head2.bmp_wide-1 ==q)
						//	sprintf(buf,"0x%02x,0x%02x",(uint8)(tp_16bit&0xff),(uint8)(tp_16bit>>8));
						//else sprintf(buf,"0x%02x,0x%02x,",(uint8)(tp_16bit&0xff),(uint8)(tp_16bit>>8));
						//write(fd,buf,strlen(buf));

                			}			
        		}
       	 		else
            			printf("read error");

			unsigned short *p1,*p2;
			unsigned int i,j,t,num=0;
			uint32 data_size;
			unsigned int widthAlignBytes;

			widthAlignBytes = ((head2.bmp_wide * 16 + 31) & ~31) / 8;
    			data_size = widthAlignBytes * head2.bmp_high;
			p1 = (unsigned short *)image_data;
    			p2 = (unsigned short *)(image_data+data_size-2);
			for (i = 0;i < data_size/2 ;i = i+2)
    			{
        			t =  *p1;
        			*p1 = *p2;
        			*p2 = t;
        			(p1)++;
        			(p2)--;
    			}
			for(i = 0;i < head2.bmp_high;i++)
    			{
       				p1 = (unsigned short *)(image_data + head2.bmp_wide*2*i);
        
        			p2 = (unsigned short *)(image_data + head2.bmp_wide*2*(i+1) -2);
        			for(j = 0;j < head2.bmp_wide/2;j++)
        			{
                			t =  *p1;
                			*p1 = *p2;
                			*p2 = t;
                			(p1)++;
                			(p2)--;
        			}
   			}

			for(k=0,image_index=0;k<head2.bmp_high;k++)
				for(q=0;q<head2.bmp_wide;q++)
				{
					if( head2.bmp_high-1 == k && head2.bmp_wide-1 ==q)
						sprintf(buf,"0x%02x,0x%02x",image_data[2*k*head2.bmp_wide+2*q],image_data[2*k*head2.bmp_wide+2*q+1]);
					else sprintf(buf,"0x%02x,0x%02x,",image_data[2*k*head2.bmp_wide+2*q],image_data[2*k*head2.bmp_wide+2*q+1]);
					write(fd,buf,strlen(buf));
					//printf ("%d %d %d %d %d\n",k,q,head2.bmp_wide,k*head2.bmp_wide+2*q,k*head2.bmp_wide+2*q+1);
				}

    		}
	}
	sprintf(buf,"};");
	write(fd,buf,strlen(buf));

	close(fd);
	printf("\n");
    return 0;
}
