/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id: color.h 50 2011-02-09 13:40:56Z spanel $
 *
 * Opravy a modifikace:
 * -
 */

#ifndef Color_H
#define Color_H

/******************************************************************************
 * Includes
 */

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Definice typu a fci pro reprezentaci RGBA barvy
 */

/* Struktura reprezentujici RGBA barvu */
typedef struct S_RGBA
{
    unsigned char   red, green, blue, alpha;
} S_RGBA;


/* Vytvori cernou barvu */
IZG_INLINE S_RGBA makeBlackColor()
{
    S_RGBA color;
    color.red = color.green = color.blue = 0;
    color.alpha = 255;
    return color;
}

/* Vytvori barvu o zadanych hodnotach */
IZG_INLINE S_RGBA makeColor(unsigned char r, unsigned char g, unsigned char b)
{
    S_RGBA color;
    color.red = r;
    color.green = g;
    color.blue = b;
    color.alpha = 255;
    return color;
}

/* Konstanty barev pouzivane v programu */
extern const S_RGBA     COLOR_BLACK;
extern const S_RGBA     COLOR_BLUE;
extern const S_RGBA     COLOR_RED;


/******************************************************************************
 * Typy a fce pro popis zdroje svetla a materialu.
 */

/* Parametry zdroje svetla */
typedef struct S_Light
{
    double  red, green, blue;   /* jednotlive barevne slozky, hodnoty 0..1 */
} S_Light;

/* Parametry materialu se zadavaji stejne jako parametry zdroje svetla */
#define S_Material S_Light

/* Vytvori strukturu nesouci nastaveni zdroje svetla */
IZG_INLINE S_Light makeLight(double r, double g, double b)
{
    S_Light light;
    light.red = r;
    light.green = g;
    light.blue = b;
    return light;
}

/* Podobna fce pro material */
#define makeMaterial makeLight

/* Konstanty materialu pouzivane v programu */
extern const S_Material     MAT_RED_AMBIENT;
extern const S_Material     MAT_RED_DIFFUSE;
extern const S_Material     MAT_RED_SPECULAR;


#ifdef __cplusplus
}
#endif

#endif // Color_H

/*****************************************************************************/
/*****************************************************************************/
