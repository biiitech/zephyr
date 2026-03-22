.. zephyr:board:: ch572dk

Overview
********

The `BIII`_ CH572DK hardware provides support for QingKe V2A 32-bit RISC-V
processor and the following devices:

* CLOCK
* :abbr:`GPIO (General Purpose Input Output)`
* :abbr:`NVIC (Nested Vectored Interrupt Controller)`

The board is equipped with two LEDs. The `BIII webpage on CH32V003`_ contains
the processor's information and the datasheet.

Hardware
********

The QingKe V2A 32-bit RISC-V processor of the BIII CH572DK is clocked by an
external crystal and runs at 48 MHz.

Supported Features
==================

.. zephyr:board-supported-hw::

Connections and IOs
===================

LED
---

* LED1 = Unconnected. Connect to an I/O pin (PD4).

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Applications for the ``ch572dk`` board target can be built and flashed
in the usual way (see :ref:`build_an_application` and :ref:`application_run`
for more details); however, an external programmer is required since the board
does not have any built-in debug support.

The following pins of the external programmer must be connected to the
following pins on the PCB (see image):

* VCC = VCC (do not power the board from the USB port at the same time)
* GND = GND
* SWIO = PD1

Flashing
========

You can use ``minichlink`` to flash the board. Once ``minichlink`` has been set
up, build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

Here is an example for the :zephyr:code-sample:`blinky` application.

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: ch572dk
   :goals: build flash

Debugging
=========

This board can be debugged via OpenOCD or ``minichlink``.

Testing the LED on the BIII CH572DK
**************************************

There is 1 sample program that allow you to test that the LED on the board is
working properly with Zephyr:

.. code-block:: console

   samples/basic/blinky

You can build and flash the examples to make sure Zephyr is running
correctly on your board. The button and LED definitions can be found
in :zephyr_file:`boards/biii/ch572dk/ch572dk.dts`.

References
**********

.. target-notes::

.. _BIII: http://www.biii-ic.com
.. _BIII webpage on CH32V003: https://www.biii-ic.com/products/CH32V003.html
