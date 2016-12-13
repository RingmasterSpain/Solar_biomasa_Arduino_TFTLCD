# Solar_biomasa_Arduino_TFTLCD
Con este código Arduino centraliza el control híbrido biomasa/solar con depósito de acumulación

* ARCHIVO: Control sistema térmico mixto biomasa-solar
 *   AUTOR: David Losada
 *   FECHA: 8/08/2016
 *     URL: http://miqueridopinwino.blogspot.com.es/2016/08/control-centralizado-del-sistema-mixto-biomasa-solar-con-Arduino.html
 *   Versión 1.3 (15/11/16)
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
 *   - Añadido código para dormir procesador cada x seg. y refresco de pantalla sólo de los valores que cambian.
 *   - Por hacer: 
      - Meter código para gestionar un 6º sensor de temperatura en intercambiador de PELLETS.
