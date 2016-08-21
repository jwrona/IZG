/******************************************************************************
 * Projekt - Zaklady pocitacove grafiky - IZG
 * spanel@fit.vutbr.cz
 *
 * $Id:$
 */

#ifndef Student_H
#define Student_H

/******************************************************************************
 * Includes
 */

#include "render.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Studentsky renderer, do ktereho muzete dopisovat svuj kod
 */

/* Jadro vaseho rendereru */
typedef struct S_StudentRenderer
{
    /* Kopie default rendereru
     * Typicky trik jak implementovat "dedicnost" znamou z C++
     * Ukazatel na strukturu lze totiz pretypovat na ukazatel
     * na prvni prvek struktury a se strukturou S_StudentRenderer
     * tak pracovat jako s S_Renderer... */
    S_Renderer  base;

    /* Zde uz muzete doplnovat svuj kod dle libosti */
    /* ??? */   

} S_StudentRenderer;


/******************************************************************************
 * Nasledujici fce budete muset re-implementovat podle sveho
 */

/* Funkce vytvori vas renderer a nainicializuje jej */
S_Renderer * studrenCreate();

/* Orezani usecky algoritmem Liang-Barsky pred vykreslenim do frame bufferu
 * Vraci -1 je-li usecka zcela mimo okno
 * x1, y1, x2, y2 - puvodni a pozdeji orezane koncove body usecky 
 * u1, u2 - hodnoty parametru u pro orezanou cast usecky <0, 1> */
int studrenCullLine(S_Renderer *pRenderer,
                    int *x1, int *y1, int *x2, int *y2,
                    double *u1, double *u2
                    );

/* Rasterizace usecky do frame bufferu s interpolaci z-souradnice
 * a zapisem do z-bufferu
 * v1, v2 - ukazatele na kocove body usecky ve 3D prostoru pred projekci
 * x1, y1, x2, y2 - souradnice koncovych bodu ve 2D po projekci
 * color - kreslici barva */
void studrenDrawLine(S_Renderer *pRenderer,
                     S_Coords *v1, S_Coords *v2,
                     int x1, int y1,
                     int x2, int y2,
                     S_RGBA color
                     );

/* Vykresli caru zadanou barvou
 * Pred vykreslenim aplikuje na koncove body cary aktualne nastavene
 * transformacni matice!
 * p1, p2 - koncove body
 * color - kreslici barva */
void studrenProjectLine(S_Renderer *pRenderer,
                        S_Coords *p1, S_Coords *p2,
                        S_RGBA color
                        );


/******************************************************************************
 * Deklarace fci, ktere jsou umistene ve student.c a pouzivaji se v main.cc
 */

/* Callback funkce volana pri startu aplikace */
void onInit();

/* Callback funkce volana pri tiknuti casovace */
void onTimer(int ticks);

/* Funkce pro vyrenderovani nacteneho modelu slozeneho z trojuhelniku */
void renderModel(S_Renderer *pRenderer, S_Model *pModel);

/* Funkce pro vyrenderovani cele sceny, tj. vykresleni modelu */
void renderScene(S_Renderer *pRenderer, S_Model *pModel);


#ifdef __cplusplus
}
#endif

#endif /* Student_H */

/*****************************************************************************/
/*****************************************************************************/
