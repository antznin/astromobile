#include <iostream>
#include <cmath>
#include <random>
#include <cstring>
#include <time.h>

#include "genMap.h"

uint32_t mod(int32_t n, uint32_t q)
{
    int r = n%q;
    if(r <0)
        return r+q;
    else 
        return r;
}

bool operator==(const rgb_t& lhs, const rgb_t& rhs)
{
    return (lhs.r == rhs.r) && (lhs.g == rhs.g) && (lhs.b == rhs.b);
}

void PathMap::getNextStep(int32_t destX, int32_t destY,
                 int32_t posCourX, int32_t posCourY,
                 int32_t &stepX, int32_t &stepY)
{
    double d = sqrt((destX-posCourX)*(destX-posCourX) + 
                      (destY-posCourY)*(destY-posCourY));

    if(d <= 100)
    {
        stepX = destX;
        stepY = destY;
        return;
    }

    float alpha = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float tmp = (MAX_STEP_DIST/static_cast <float> (d));

    if(tmp < 1)
    {
        alpha = alpha * tmp;
    }


    if(alpha>0.5)
    {
        alpha = 1 - alpha;
    }

    float beta = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*d*alpha))) - d*alpha;

    stepX = alpha*(destX-posCourX) + posCourX;

    stepY = alpha*(destY-posCourY) + posCourY;

    stepX += static_cast <float> (beta*(posCourY-destY))/static_cast <float> (d);
    stepY += beta*(destX-posCourX)/d;

    this->paintRoad(posCourX, posCourY, stepX, stepY);
}

/* Generates a new destination */
void PathMap::genDest(coord_t actualPos, coord_t &dest)
{
    float randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    dest.x = (int) (actualPos.x + NEW_DEST_MAX_DIST_M*randVal/3);
    randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    dest.y = (int) (actualPos.y + NEW_DEST_MAX_DIST_M*randVal/3);

    for(int i = -20; i <= 20; ++i)
	{
		for(int j = -20; j <= 20; ++j)
		{
			setPx(dest.x/PX_TO_M + i, dest.y/PX_TO_M + j , {255,0,0});
		}
	}
}

/* Computes the position of the closest station */
void PathMap::getClosestStation(coord_t actualPos, coord_t &stationPos)
{
    float randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);

    stationPos.x = actualPos.x + STATION_MAX_DIST_M*randVal/PX_TO_M/3;
    randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    stationPos.y = actualPos.y + STATION_MAX_DIST_M*randVal/PX_TO_M/3;

    for(int i = -15; i <= 15; ++i)
	{
		for(int j = -15; j <= 15; ++j)
		{
			setPx(stationPos.x/PX_TO_M + i, stationPos.y/PX_TO_M + j , {0,0,255});
		}
	}

    this->paintRoad(actualPos.x, actualPos.y, stationPos.x, stationPos.y);
}

/* Generates a new waypoint between actual position and the destination*/
void PathMap::genWp(coord_t actualPos, coord_t dest, coord_t &wp)
{
    int32_t stepX;
    int32_t stepY;

    getNextStep((int)dest.x, (int) dest.y,
                (int) actualPos.x, (int) actualPos.y,
                stepX, stepY);
    wp.x = (double) stepX;
    wp.y = (double) stepY;
}

PathMap::PathMap()
{
	srand (time(NULL));
	this->img = new rgb_t*[IMG_H_PX];
	for(int i = 0; i < IMG_H_PX; ++i)
	{
		this->img[i] = new rgb_t[IMG_W_PX];
		for(int j = 0; j < IMG_W_PX; ++j)
		{
			this->img[i][j] = BLANK_RGB;
		}
	}
}

PathMap::~PathMap() 
{
    for(int i = 0; i < IMG_H_PX; ++i)
    {
        delete[] this->img[i];
    }
    delete[] this->img;
}

void PathMap::setPx(int32_t coordX, int32_t coordY, rgb_t val)
{
    this->img[mod(coordY,IMG_H_PX)][mod(coordX,IMG_W_PX)] = val;
}

rgb_t PathMap::getPx(int32_t coordX, int32_t coordY){
    return this->img[mod(coordY,IMG_H_PX)][mod(coordX,IMG_W_PX)];
}

void PathMap::drawDisc(int32_t coordX, int32_t coordY, rgb_t value)
{
    for(int i = -1; i <= 1; ++i)
    {
        for(int j = -2; j <= 2; ++j)
        {
            setPx(coordX + i, coordY + j , value);
        }
        setPx(coordX - 2, coordY + i, value);
        setPx(coordX + 2, coordY + i, value);
    }
}

void PathMap::paintRoad(int32_t sourceX, int32_t sourceY, 
                        int32_t destX, int32_t destY)
{
    int32_t xBeg, xEnd;
    int32_t yLow, yHi;
    if(sourceX < destX)
    {
        xBeg = sourceX/PX_TO_M;
        xEnd = destX/PX_TO_M;
    }
    else
    {
        xBeg = destX/PX_TO_M;
        xEnd = sourceX/PX_TO_M;
    }
    if(sourceY < destY)
    {
        yLow = sourceY/PX_TO_M;
        yHi = destY/PX_TO_M;
    }
    else
    {
        yLow = destY/PX_TO_M;
        yHi = sourceY/PX_TO_M;
    }

    double slope;
    if(destX == sourceX)
        slope = 0;
    else
        slope = static_cast<double>(destY - sourceY) / static_cast<double> (destX - sourceX);



    for(int x= xBeg; x <= xEnd; ++x)
    {
        int32_t y0 = slope * (x - 0.5 - sourceX/PX_TO_M) + sourceY/PX_TO_M;
        int32_t y1 = slope * (x + 0.5 - sourceX/PX_TO_M) + sourceY/PX_TO_M;
        if(y1 < y0)
        {
            uint32_t tmp = y1;
            y1 = y0;
            y0 = tmp;
        }
        if(y0 < yLow)
            y0 = yLow;
        if(y1 > yHi)
            y1 = yHi;


        for(int32_t y = y0; y <= y1; ++y)
        {
            drawDisc(x, y, ASPHALT_RGB);
        }

//        int32_t y = slope * (x - sourceX/PX_TO_M) + sourceY/PX_TO_M;

        //takePhoto({(double)x, (double)y});
    }
}

void PathMap::dumpImage(const char* file_name)
{
    FILE *f;

    int w = IMG_W_PX;
    int h = IMG_H_PX;

    int x, y, r, g, b;

    unsigned char *img = NULL;
    int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

    img = (unsigned char *)malloc(3*w*h);
    memset(img,0,3*w*h);

    for(int i=0; i<w; i++)
    {
        for(int j=0; j<h; j++)
        {
            x=i; y=(h-1)-j;
            rgb_t pixel = this->img[x][y];

            r = pixel.r;
            g = pixel.g;
            b = pixel.b;
            img[(x+y*w)*3+2] = (unsigned char)(r);
            img[(x+y*w)*3+1] = (unsigned char)(g);
            img[(x+y*w)*3+0] = (unsigned char)(b);
        }
    }

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(       w    );
    bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(       w>>16);
    bmpinfoheader[ 7] = (unsigned char)(       w>>24);
    bmpinfoheader[ 8] = (unsigned char)(       h    );
    bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
    bmpinfoheader[10] = (unsigned char)(       h>>16);
    bmpinfoheader[11] = (unsigned char)(       h>>24);

    f = fopen(file_name,"wb");
    fwrite(bmpfileheader,1,14,f);
    fwrite(bmpinfoheader,1,40,f);
    for(int i=0; i<h; i++)
    {
        fwrite(img+(w*(h-i-1)*3),3,w,f);
        fwrite(bmppad,1,(4-(w*3)%4)%4,f);
    }

    free(img);
    fclose(f);
}

uint8_t PathMap::getTextureFromPx(rgb_t px)
{
    if(px.g < 105)
        return 2;
    else if(px.g < 140)
        return 0;
    else
        return 1;
}

void PathMap::getProbWeights(int32_t coordX, int32_t coordY, float proba[3])
{
    memcpy(proba, this->defaultTextureProba, 3*sizeof(float));

    if(getPx(coordX, coordY) ==  (rgb_t) BLANK_RGB)
    {
        float tmp[] = {0,0,0};
        memcpy(proba, tmp, 3*sizeof(float));
    }
    else if((getPx(coordX, coordY) ==  (rgb_t) WATER_RGB) ||
    		(getPx(coordX, coordY) ==  (rgb_t) GRASS_RGB) ||
			(getPx(coordX, coordY) ==  (rgb_t) TREE_RGB))
    {
        uint8_t textureId = getTextureFromPx(getPx(coordX, coordY));
        for(int i = 0; i < 3; ++i)
        {
            proba[i] = (1 - this->probaIfSame) * proba[i]/(1-proba[textureId]);
        }
        proba[textureId] = this->probaIfSame;
    }
}

uint8_t PathMap::decideTexture(float probWeights[3])
{
    float randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    float tmp = 0;
    for(uint8_t i = 0; i < 3; ++i)
    {
        tmp += probWeights[i];
        if(randVal < tmp)
            return i;
    }
    return -1;
}

void PathMap::genPx(int32_t coordX, int32_t coordY)
{
    if(!(getPx(coordX, coordY) == (rgb_t) BLANK_RGB))
        return;

    float proba[3] = {0,0,0};
    float tmp[3] = {0,0,0};
    
    float sum = 0;

    getProbWeights(coordX-1, coordY-1, tmp);
    for(uint8_t i = 0; i < 3; ++i)
    {
        proba[i] += tmp[i];
        sum += tmp[i];
    }
    getProbWeights(coordX-1, coordY  , tmp);
    for(uint8_t i = 0; i < 3; ++i)
    {
        proba[i] += tmp[i];
        sum += tmp[i];
    }
    getProbWeights(coordX-1, coordY+1, tmp);
    for(uint8_t i = 0; i < 3; ++i)
    {
        proba[i] += tmp[i];
        sum += tmp[i];
    }
    getProbWeights(coordX  , coordY-1, tmp);
    for(uint8_t i = 0; i < 3; ++i)
    {
        proba[i] += tmp[i];
        sum += tmp[i];
    }
    getProbWeights(coordX  , coordY+1, tmp);
    for(uint8_t i = 0; i < 3; ++i)
    {
        proba[i] += tmp[i];
        sum += tmp[i];
    }
    getProbWeights(coordX+1, coordY-1, tmp);
    for(uint8_t i = 0; i < 3; ++i)
    {
        proba[i] += tmp[i];
        sum += tmp[i];
    }
    getProbWeights(coordX+1, coordY  , tmp);
    for(uint8_t i = 0; i < 3; ++i)
    {
        proba[i] += tmp[i];
        sum += tmp[i];
    }
    getProbWeights(coordX+1, coordY+1, tmp);

    if(sum == 0)
        memcpy(proba, this->defaultTextureProba, 3*sizeof(float));
    else
    {
        for(uint8_t i = 0; i < 3; ++i)
        {
            proba[i] /= sum;
        }
    }
    setPx(coordX, coordY, this->textures[decideTexture(proba)]);
}

rgb_t PathMap::takePhoto(coord_t position)
{
	int32_t x = (int32_t) position.x / PX_TO_M;
	int32_t y = (int32_t) position.y / PX_TO_M;

	for(int32_t i = 0; i < 25; ++i)
	{
		for(int32_t j = 0; j < 25; ++j)
		{
			genPx(x+i, y+j);
			genPx(x+i, y-j);
			genPx(x-i, y+j);
			genPx(x-i, y-j);
		}
	}

	rgb_t avgColour;

	for(int32_t loopId = 0; loopId < 10000/SIMU_ACCEL; loopId++)
	{
		uint32_t sumR = 0;
		uint32_t sumG = 0;
		uint32_t sumB = 0;

		rgb_t colour0;
		rgb_t colour1;
		rgb_t colour2;
		rgb_t colour3;



		for(int32_t i = 0; i < 25; ++i)
		{
			for(int32_t j = 0; j < 25; ++j)
			{
				colour0 = getPx(x+i, y+j);
				colour1 = getPx(x+i, y-j);
				colour2 = getPx(x-i, y+j);
				colour3 = getPx(x-i, y-j);
				sumR += colour0.r + colour1.r + colour2.r + colour3.r;
				sumG += colour0.g + colour1.g + colour2.g + colour3.g;
				sumB += colour0.b + colour1.b + colour2.b + colour3.b;
			}
		}
		avgColour = {(uint8_t)(sumR/(25*25*4)), (uint8_t)(sumG/(25*25*4)), (uint8_t)(sumB/(25*25*4))};
	}
	return avgColour;
}

//int main(int argc, char const *argv[])
//{
//	PathMap pm;
//
//    coord_t posCour = {IMG_W_M/2,IMG_H_M/2};
//    std::cout << "posCour: " <<  posCour.x << "," << posCour.y << std::endl;
//
//    coord_t dest;
//    coord_t step;
//
//    for(int i=0; i<5; i++)
//    {
//    	pm.genDest(posCour, dest);
//    	std::cout << "dest: " <<  dest.x << "," << dest.y << std::endl;
//		while(step.x != dest.x && step.y != dest.y)
//		{
//			pm.genWp(posCour, dest, step);
//			posCour.x = step.x;
//			posCour.y = step.y;
//		}
//    }
//    pm.getClosestStation(posCour, step);
//
//    pm.dumpImage("test.bmp");
//
//    return 0;
//}
