#include "tracking.h"



/* 原图是YUYV图 目的图是二值图
 */
void threshold(u8 image_src[],u8 image_dst[],u32 width,u32 height,u32 *centroid_x,u32 *centroid_y,u8 t){
    /* if pn < ((gs(n) + gs(n - w))/(2 * s)) * ((100 - t)/100)  则输出0
     * else 输出1
     * 其中，gs(n) = gs(n - 1) * (1 - 1/s) + pn
     *       gs(n - w) = gs - value of pixel above current pixel
     */

    int i,x,x_src,y,yh;
		u32 sum_x = 0,sum_y = 0,sum_area = 0;
    u32 s = width >> 4;  //需要计算的前面的点数s是图像宽度的八分之一最合适
    const u32 S = 9;
    const u32 power2S = 1 << S; //为了加快运算速度，与power2s相乘的数可避免浮点数的运算
    u32 factor = power2S*(100 - t)/(100 * s);
    u32 q = power2S - power2S/s;
    u32 gn = 127 * s;  //设定初始gn，127为像素值255的中间值
    u32 pn,hn;
    u32 *prev_gn = NULL;
    u8 *scanline = NULL;

    prev_gn = (u32 *)malloc(width * sizeof(u32));  //给历史gn数组分配内存,其长度为图像的宽度
    for(i = 0;i < width;i++){
        prev_gn[i] = gn;  //初始化历史gn数组
    }
    for(y = 0;y < height;y++){  //从行始扫到行末
        yh = y * width;
        scanline = image_src + (yh<<1);
        for(x = 0;x < width;x++){
            x_src = x << 1;  //x = 2*x;
            pn = scanline[x_src];
            gn = ((gn * q) >> S) + pn;  //gs(n) = gs(n - 1) * (1 - 1/s) + pn
            hn = (gn + prev_gn[x]) >> 1;  //gs(n) + gs(n - w)/2
            prev_gn[x] = gn;
						if(pn < ((hn * factor) >> S)){
								image_dst[yh + x] = 0 ; 
								sum_x = sum_x + x; //计算白色区域横坐标的总和
								sum_y = sum_y + y; //计算白色区域纵坐标的总和
								sum_area++;   //计算白色区域像素点数目
						}else{
								image_dst[yh +x] = 0xff;  //进行二值化
						}
        }
        y++;
        if(y == height){
            break;
        }
        yh = y * width;  //yh = y*width
        scanline = image_src + (yh<<1);
        for(x = width - 1;x >= 0;x--){
            x_src = x << 1;  //x = 2*x;
            pn = scanline[x_src];
            gn = ((gn * q) >> S) + pn; //gs(n) = gs(n - 1) * (1 - 1/s) + pn
            hn = (gn + prev_gn[x]) >> 1;  //gs(n) + gs(n - w)/2
            prev_gn[x] = gn;
            if(pn < ((hn * factor) >> S)){
								image_dst[yh + x] = 0 ; 
								sum_x = sum_x + x; //计算白色区域横坐标的总和
								sum_y = sum_y + y; //计算白色区域纵坐标的总和
								sum_area++;   //计算白色区域像素点数目
						}else{
								image_dst[yh +x] = 0xff;  //进行二值化
						}
        }
    }
		*centroid_x = sum_x/sum_area;  //计算白色区域的质心横坐标
		*centroid_y = sum_y/sum_area;  //计算白色区域的质心纵坐标
    free(prev_gn);

}

void series_act(u8 image[]){
	
	
//	threshold((u8 *)g_camera_image,image_thresh,100,100,&centroid_x,&centroid_y,2);
    
}
