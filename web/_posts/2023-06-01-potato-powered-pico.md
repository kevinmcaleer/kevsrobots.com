---
layout: blog
title: Potato Powered Pico
description: "Power from Potatoes: An Introduction to Potato Batteries"
short_title: Potato powered Pico
short_description: Intro to Potato Batteries
date: 2023-06-01
author: Kevin McAleer
excerpt: 
cover: /assets/img/blog/potato-power/potato.jpg
tags: 
 - Raspberry Pi
 - Pico
 - Potato
 - battery
 - DIY battery
groups:
 - weird
 - garden
---

The humble `potato`, a staple of many diets around the world, has a surprising additional use: as a battery. Today, we'll explore the science behind potato batteries, perform some calculations, and walk you through how to wire them together.

## Can you power a Raspberry Pi Pico with 10 potatoes?

I experimented with creating a Potato Powered Pico, however 10 Potatos with Zinc and Copper plates didn't provide enough amps.

{% include youtubeplayer.html id="UURmlI0Sru0" %}

---

## Potato Batteries: A Science Lesson

The potato battery is a type of electrochemical cell, or a device that creates electricity through a chemical reaction. Despite its name, the power isn't coming directly from the potato, but rather from the chemical reaction taking place between two [electrodes](/resources/glossary#electrode) inserted into the potato and the potato's juice, which acts as the electrolyte.

Typically, the two electrodes used are zinc and copper. The zinc electrode (usually a galvanized nail) undergoes oxidation, meaning it loses electrons. The copper electrode (often a copper penny or wire) undergoes reduction, meaning it gains electrons. The potato's juice facilitates the movement of these electrons from the zinc electrode to the copper electrode. This movement of electrons is what we call electricity.

---

## Calculating Voltage and Amperage

The voltage, or electric potential difference, between the two electrodes in a potato battery is determined by the nature of the materials used. This is described by the [Nernst](#the-nernst-equation) equation, a principle of electrochemistry. However, in a simple potato battery with a zinc and copper electrode, the voltage generated typically averages around 0.5 to 0.9 volts.

As for current, or amperage, it's generally quite low, typically around a few milliamperes (thousandths of an ampere). The current depends on factors such as the size of the electrodes, their distance apart, and the quantity and quality of the electrolyte (in this case, the potato juice). Remember, this isn't a high-performance power source, but rather a fun and educational science experiment.

---

## Wiring Potato Batteries Together

To increase the voltage output, you can wire multiple potato batteries in series. This means connecting the copper electrode of one potato to the zinc electrode of the next with a wire. The total voltage then becomes the sum of the voltage of all individual potato batteries.

For example, if one potato battery gives 0.8 volts, ten potato batteries connected in series would yield 0.8 * 10 = 8 volts.

To increase current, you would wire multiple potato batteries in parallel. This means connecting all the zinc electrodes together and all the copper electrodes together. The total current becomes the sum of the currents of all individual potato batteries.

For example, if one potato battery produces a current of 2 milliamperes, ten potato batteries connected in parallel would yield 2 * 10 = 20 milliamperes.

However, it's essential to remember that a potato battery's output is quite limited, even with multiple potatoes wired together. You could light a small LED or power a low-power digital clock, but don't expect to charge your smartphone!

---

## In Conclusion

Potato batteries offer a unique and fun way to learn about the principles of electrochemistry and circuits. They also serve as a reminder that the principles of electricity are embedded all around us—even within the humble potato.

Yet, while potato batteries won't be powering our homes anytime soon, they certainly light the spark of scientific curiosity. As we continue to search for innovative and sustainable energy sources, who knows what other surprising power-generating methods we'll uncover?

> ## The Nernst Equation
>
>The Nernst Equation is a fundamental concept in electrochemistry, formulated by a German chemist named Walther Nernst in the late 19th century.
>
>Born in 1864, Walther Nernst was a pioneering scientist who made substantial contributions to the field of physical chemistry. He was awarded the Nobel Prize in Chemistry in 1920 for his work on thermochemistry, particularly for his development of the third law of thermodynamics.
>
>The Nernst Equation is one of his most recognized scientific contributions. It's an equation that relates the reduction potential of an electrochemical reaction (also known as the voltage or electric potential difference) to the standard electrode potential, temperature, and the activities of the chemical species undergoing reduction and oxidation. 
>
>The equation is often stated as:
>
> E = E₀ - (RT/nF) * ln(Q)
>
>Here:
>
> - E is the cell potential, or voltage, at non-standard conditions (that is, conditions other than 25°C or other than 1M concentration).
> - E₀ is the cell potential at standard conditions.
> - R is the universal gas constant.
> - T is the absolute temperature in Kelvin.
> - n is the number of moles of electrons exchanged in the electrochemical reaction.
> - F is the Faraday constant (the magnitude of charge per mole of electrons).
> - Q is the reaction quotient, which is the ratio of the concentrations (or activities) of the products divided by those of the reactants.
>
> In the context of the potato battery, the Nernst Equation provides the theoretical foundation for understanding why the battery generates voltage. The zinc and copper electrodes have different tendencies to lose or gain electrons, and this difference, as quantified by the Nernst Equation, leads to the generation of voltage.
>
> However, applying the Nernst Equation in practice often involves simplifying assumptions, and it's worth noting that the real-world performance of a potato battery may not perfectly match the theoretical predictions due to factors like internal resistance, non-ideal behavior of the potato juice as an electrolyte, and non-uniform contact between the electrodes and the potato.

---
