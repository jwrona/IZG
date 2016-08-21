/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: color.c 50 2011-02-09 13:40:56Z spanel $
 *
 * Opravy a modifikace:
 * -
 */

#include "color.h"

/****************************************************************************
 * Globalni promenne
 */

/* Preddefinovane barvy */
const S_RGBA    COLOR_BLACK = { 0, 0, 0, 255 };
const S_RGBA    COLOR_BLUE  = { 0, 0, 255, 255 };
const S_RGBA    COLOR_RED   = { 255, 0, 0, 255 };

/* Materialy */
const S_Material    MAT_RED_AMBIENT  = { 0.8, 0.2, 0.2 };
const S_Material    MAT_RED_DIFFUSE  = { 0.8, 0.2, 0.2 };
const S_Material    MAT_RED_SPECULAR = { 0.8, 0.8, 0.8 };


/*****************************************************************************/
/*****************************************************************************/
