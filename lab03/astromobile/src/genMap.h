#ifndef __GEN_MAP_H__
#define __GEN_MAP_H__

#define SIMU_ACCEL 30
#define SIMU_TIME 60 // seconds

#define IMG_W_PX 7000
#define IMG_H_PX 7000

#define PX_TO_M 10

#define IMG_W_M (PX_TO_M * IMG_W_PX)
#define IMG_H_M (PX_TO_M * IMG_H_PX)

#define NEW_DEST_MAX_DIST_M 50000
#define STATION_MAX_DIST_M 5000

#define NEW_DEST_MAX_DIST_PX = NEW_DEST_MAX_DIST_M / PX_TO_M


#define MAX_STEP_DIST 5000.0

#define BLANK_RGB {0,0,0}
#define ASPHALT_RGB {163, 161, 165}
#define WATER_RGB {60,120,230}
#define GRASS_RGB {160,160,80}
#define TREE_RGB {80,90,80}

#define TEXTURES {WATER_RGB, GRASS_RGB, TREE_RGB}
#define TEXTURES_PROB {0.1,0.3,0.6}
#define PROBA_SAME 0.8

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

typedef struct
{
    double x;
    double y;
} coord_t;


class PathMap {
    public:
        float defaultTextureProba[3] = TEXTURES_PROB;
        float probaIfSame = PROBA_SAME;
        rgb_t textures[3] = TEXTURES;

        PathMap();
        ~PathMap();

        rgb_t takePhoto(coord_t position);
        void genWp(coord_t actualPos, coord_t dest, coord_t &wp);
        void genDest(coord_t actualPos, coord_t &dest);
        void getClosestStation(coord_t actualPos, coord_t &stationPos);
        void dumpImage(const char* file_name);

    private:
        rgb_t **img;

        void setPx(int32_t coordX, int32_t coordY, rgb_t val);
		rgb_t getPx(int32_t coordX, int32_t coordY);
		void drawDisc(int32_t coordX, int32_t coordY, rgb_t value);
		void paintRoad(int32_t sourceX, int32_t sourceY,
					   int32_t destX, int32_t destY);
		void getNextStep(int32_t destX, int32_t destY,
						 int32_t posCourX, int32_t posCourY,
						 int32_t &stepX, int32_t &stepY);
        void getProbWeights(int32_t coordX, int32_t coordY, float proba[3]);
        void genPx(int32_t coordX, int32_t coordY);
        uint8_t decideTexture(float probWeights[3]);
        uint8_t getTextureFromPx(rgb_t px);
};

#endif
