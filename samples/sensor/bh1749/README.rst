.. _bh1749:

BH1749: Ambient Light Sensor IC
#####################################################

Description
***********

This sample application sets up the BH1749 color sensor to provide 8-bit
measurement data every time a set threshold value (>50) is reached for the
RED color channel.
It also shows how to enable interrupt every time data is ready, instead
of threshold trigging.

References
**********

 - BH1749NUC-E: https://www.rohm.com/datasheet/BH1749NUC/bh1749nuc-e

Building and Running
********************

This project outputs sensor data to the console. It requires a BH1749
sensor. It should work with any platform featuring a I2C peripheral interface.
It does not work on QEMU.
In this example below the :ref:`nrf91_pca20035` board is used.


.. zephyr-app-commands::
   :zephyr-app: samples/sensors/bh1749
   :board: nrf91_pca20035
   :goals: build flash

Sample Output
=============

.. J-Link RTT Viewer

    device is 0x20022384, name is BH1749

    Threshold trigger

    BH1749 RED: 387

    BH1749 GREEN: 753

    BH1749 BLUE: 397

    BH1749 IR: 81

    (continues when trigger level reached)

