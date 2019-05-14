#ifndef STRUCTS_H
#define STRUCTS_H
#include "header.h"

struct contenedor_de_parametros_de_control
{
    //Posicion del objeto respecto el sistema
    //de coordenadas de referencia en el instante cero
    double GX0;
    double GY0;
    double GZ0;
    //Orientacion del objeto respecto el sistema
    //de coordenadas de referencia en el instante cero
    double RX0;
    double RY0;
    double RZ0;

    //Posicion del objeto respecto el sistema
    //de coordenadas de referencia en el instante uno
    double GX1;
    double GY1;
    double GZ1;
    //Orientacion del objeto respecto el sistema
    //de coordenadas de referencia en el instante uno
    double RX1;
    double RY1;
    double RZ1;
    //Posicion del punto H respecto al sistema
    //de coordenadas del objeto
    double HR;
    double HS;
    double HT;

    double dis_focal;

};

//La siguiente definicion describe el contenedor que
//usaremos para guardar resultados

struct contenedor_de_resultados
{
    //Matriz de rotacion. Dos formas equivalentes de definirla
    double R0[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante cero
    double R1[9]; //Fijo, mediante arreglo. La reserva y liberacion de memoria es automatica para el instante uno
    //Posicion del punto H respecto al sistema
    //de coordenadas de referencia para el instante cero
    double HX0;
    double HY0;
    double HZ0;

    //Posicion del punto H respecto al sistema
    //de coordenadas de referencia para el instante 1
    double HX1;
    double HY1;
    double HZ1;

    //Posición bidimensional del punto en el plano de la cámara en el instante cero
    double HX00;
    double HY00;
    //Posición bidimensional del punto en el plano de la cámara en el instante uno
    double HX11;
    double HY11;
    //vector de flujo óptico entre los instantes de tiempo 0 y 1
    double dx;
    double dy;

};

//***
//***Insertar aqui las definiciones de variables globales,
//***que son aquellas variables que se podran acceder desde
//***cualquier funcion dentro de este archivo
//***

//El siguiente puntero global apuntara al contenedor que
//usaremos para guardar los valores de control de flujo
//del programa que se leeran de un archivo de texto

struct contenedor_de_parametros_de_control *p_parametros;

//El siguiente puntero global apuntara al contenedor
//que usaremos para guardar resultados

struct contenedor_de_resultados *p_resultados;

//La siguiente variable global se usara como contador
//el numero de datos leidos

int numeroDeDatosLeidos=0;

//***
//***Insertar aqui las constantes del programa
//***

#define PI 3.141592652
#define DESPLIEGUE_MATRIZ_DE_ROTACION 1 //1: si, 0: no



#endif
