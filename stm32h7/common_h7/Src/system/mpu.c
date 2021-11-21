#include "config/config.h"
#include "hardware.h"
#include "system/mpu.h"
#include "system/hw_util.h"
#include "debug_helper.h"

/* Definition of non cached memory areas */
typedef struct {
    uint32_t        regionAddress;
    MPU_AreaType    regionType;
    uint32_t        regionSize;
    uint8_t         regionNum;
    uint8_t         regionActive;                   /* flag for "region is valid and activated" */
} MPURegionT;

#define MAX_MPU_REGIONS     4                       /* Number of defined MPU regions we can handle */
#define PWROF2(a)           ( (a & (a-1)) == 0 ) 

static MPURegionT mpuRegions[MAX_MPU_REGIONS]; 
static uint32_t   mpuRegionsUsed = 0;
/******************************************************************************
 * Add a new MPU region, defined by startaddress, type and size.
 * The following constraints will be checked
 *   - the size must be any square of 2 in the range 32 to 4 GB
 *   - the type must be any of MPU_AreaType enum
 *   - the start address must be aligned to the size
 * an error code is returned, if any of these constraints is violated
 * If there is no more space left to add another region, an error will be
 * returned, too
 *
 * MPUERR_OK ( 0 ) will be returned on success
 *****************************************************************************/
int32_t MPU_AddRegion ( uint32_t rgnAddr, MPU_AreaType rgnType, uint32_t rgnSize, uint8_t rgnNum )
{
    /* check constraints - size */
    if ( rgnSize < 32 || !PWROF2(rgnSize) ) return MPUERR_ILLEGAL_SIZE;

    /* check constraints - type */
    if ( rgnType >= MPUTYPE_FLASH___MAX )  return MPUERR_ILLEGAL_TYPE;

    /* check constraints - address */
    if ( (rgnAddr & (rgnSize-1)) != 0 )  return MPUERR_ILLEGAL_ADDR;

    /* check constraints - region */
    #if defined ( CORE_CM7 )
        if ( rgnNum >= 16 )  return MPUERR_ILLEGAL_NUM;
    #elif defined ( CORE_CM4 )
        if ( rgnNum >= 8 )  return MPUERR_ILLEGAL_NUM;
    #else
        return MPUERR_ILLEGAL_NUM;
    #endif

    /* constraints are ok - check free space */
    if ( mpuRegionsUsed == MAX_MPU_REGIONS ) return MPUERR_NOSPACELEFT;

    /* Add entry */
    MPURegionT *ptr    = mpuRegions+mpuRegionsUsed;
    ptr->regionAddress = rgnAddr;
    ptr->regionType    = rgnType;
    ptr->regionSize    = rgnSize;
    ptr->regionNum     = rgnNum;
    ptr->regionActive  = 0;
    mpuRegionsUsed++;

    return MPUERR_OK;
}

/******************************************************************************
 * Setup the MPU region parameters according to the passed region type
 * see AN 4838 for details
 *
 * Return values:
 *   0 will be returned in case of successful initialization
 *  -1 in case of any failure
 *****************************************************************************/
static int32_t MPU_SetupRegion ( MPU_Region_InitTypeDef *init, MPU_AreaType rgnType )
{
    /* 
     * common default settings for most configurations,
     * will be overwritten, if not matching
     */
    init->AccessPermission = MPU_REGION_FULL_ACCESS;
    init->SubRegionDisable = 0x00;
    init->TypeExtField     = MPU_TEX_LEVEL0;
    init->DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

    switch ( rgnType ) {
        case MPUTYPE_RAM_DEVICEMEM_SHARED:
            /* device memory, shareable */
            init->IsBufferable     = MPU_ACCESS_BUFFERABLE;
            init->IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
            init->IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
            break;
        case MPUTYPE_RAM_NONCACHEABLE:
            /* uncacheable RAM */
            init->IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
            init->IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
            init->IsShareable      = MPU_ACCESS_SHAREABLE;
            init->TypeExtField     = MPU_TEX_LEVEL1;
            break;
        case MPUTYPE_RAM_WT:
            /* cacheable, write-thru RAM, no write allocate */
            init->IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
            init->IsCacheable      = MPU_ACCESS_CACHEABLE;
            init->IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
            break;
        case MPUTPYE_FLASH_NOWRITE:
            /* write protected flash */
            init->AccessPermission = MPU_REGION_PRIV_RO_URO;
            init->IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
            init->IsCacheable      = MPU_ACCESS_CACHEABLE;
            init->IsShareable      = MPU_ACCESS_SHAREABLE;
            break;
        case MPUTYPE_FLASH_WRITE:
            /* writeable flash */
            init->IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
            init->IsCacheable      = MPU_ACCESS_CACHEABLE;
            init->IsShareable      = MPU_ACCESS_SHAREABLE;
            break;
        default: return -1;
    }

    return 0;
}

/******************************************************************************
 * configure the previously defined MPU regions
 *****************************************************************************/
void MPU_EnableAllRegions(void)
{
    MPU_Region_InitTypeDef Init;
    MPURegionT *ptr;

    /* Any regions at all? */
    if ( mpuRegionsUsed == 0 ) return;

    /* Disable the MPU */
    HAL_MPU_Disable();

    for ( uint32_t i = 0; i < mpuRegionsUsed; i++ ) {
        ptr = mpuRegions+i;
        /* 
         * First fill in the type parameters. 
         * Only if successful, continue in setting up this range
         */
        if ( MPU_SetupRegion(&Init, ptr->regionType) == 0 ) {
            Init.BaseAddress  = ptr->regionAddress;
            Init.Size         = HW_GetLn2(ptr->regionSize) - 1;
            Init.Number       = ptr->regionNum;
            Init.Enable       = MPU_REGION_ENABLE;
            HAL_MPU_ConfigRegion(&Init);
            ptr->regionActive = 1;
        } // if

    } // for

    /* Enable the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

const char * const mputype_txt[]=MPUTYPE_TEXTS;

static const char * MPU_GetTypeName(MPU_AreaType    regionType )
{
    if ( regionType < sizeof(mputype_txt)/sizeof(char *) )
        return mputype_txt[regionType];
    else
        return "???";
}

/******************************************************************************
 * dump info about every configured MPU region
 *****************************************************************************/
void MPU_Dump(void)
{
    MPURegionT *ptr;
    DEBUG_PRINTF("MPU Regions\n"); 
    DEBUG_PRINTF("Num    Address       Size Type\n"); 
    DEBUG_PRINTF("---!----------!----------!-----------------------\n"); 
    for ( uint32_t i = 0; i < mpuRegionsUsed; i++ ) {
        ptr = mpuRegions+i;
        DEBUG_PRINTF("%3d ", ptr->regionNum);
        if ( ptr->regionActive ) {
            DEBUG_PRINTF("0x%p 0x%08x %s\n", ptr->regionAddress, ptr->regionSize, MPU_GetTypeName(ptr->regionType)); 
        } else {
            DEBUG_PRINTF("Invalid region\n"); 
        }
    } // for
}
