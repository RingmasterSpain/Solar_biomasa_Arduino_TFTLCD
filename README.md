# Solar_biomasa_Arduino_TFTLCD
Con este código Arduino centraliza el control híbrido biomasa/solar con depósito de acumulación

* ARCHIVO: Control sistema térmico mixto biomasa-solar
 *   AUTOR: David Losada
 *   FECHA: 8/08/2016
 *   URL: http://miqueridopinwino.blogspot.com.es/2016/08/control-centralizado-del-sistema-mixto-biomasa-solar-con-Arduino.html
 *   Versión 1.8 (7/05/18)
 *   - Se ajustan las temperaturas de conexión y desconexión biomasa y captador para evitar perder energía en tubos por desajustes
 *   - Se ha corregido el código de desconexión en caso de biomasa encendida, para evitar apagar el motor demasiado pronto con brasas
 *   - Se añade aviso de excesiva diferencia entre sondas, indicando en rojo la temperatura y activando alarma
 *   - Mejorada la alarma; se hace intermitente.
 *   - Añadido un 5º termistor en el serpentin biomasa (parte baja) por seguridad; Biomasa2, a veces se calienta antes en la parte inferior
 *   - Corregida la contabilización de horas funcionamiento
 *   - Cambiado el orden en pantalla
 *   - Se indican los KWh ahorrados por energía renovable :) (aproximado)
 *   - Cambiada la forma en que se comprueban temperaturas, dando prioridad real a biomasa (o estamos jodidos)
 *   - Ahora se activa cada 15 días la válvula de enfriado para evitar su agarrotamiento
 *   - Corregidos varios errores
 *   - 10/12/16 Mejorados varios puntos; no contabilizaba horas de motor, y la comprobación de temperaturas no era óptima con depósito
 *   - 12/12/16 Mejorado el código: añadido código para dormir el procesador, reducir los refrescos a lo mínimo necesario y sensor de pellets futuro
 *   - 15/12/16 Corregida la evaluación del tiempo
 *   - 07/01/16 Corregido error en la condición de apagado del motor en caso de activación por el captador solar
 *	 - 18/01/17 Añadido tiempo mínimo de activación motor (se estaba activando intermitentemente en determinados casos)
 *   - 30/01/17 Añado la condición de que estén los sensores de biomasa al menos a cierta temp. para activar el motor, ya que puede que se calienten por el sol en invierno
 *   - 20/03/17 Ajustados valores; con el fuego al principio difTemp era demasiado pequeña.
 *   - 05/05/17 Eliminado el ajuste mínimo de temperatura de activación, era peor el remedio que la enfermedad.
 *   - 07/05/18 Al ver que en caso de mucha temperatura de la biomasa, las diferencias entre temperaturas son mínimas, lo tengo en cuenta ahora al disminuir la temperatura de activación según ésta sube
 *   - Por hacer: 
 *    - Meter código para gestionar un 6º sensor de temperatura en intercambiador de PELLETS.
