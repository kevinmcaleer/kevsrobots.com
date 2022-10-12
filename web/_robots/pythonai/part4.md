---
layout: pythonai
title: Part 4 - Weather Skill
subtitle: Lets build our next skill - get the weather forecast for our location
thanks: true
description: Lets build our next skill - get the weather forecast for our location
excerpt: Lets build our next skill - get the weather forecast for our location
video: txKJmd2P_e4
code: https://www.github.com/kevinmcaleer/pythonai
---

### Table of Contents

{:toc}
* toc

---

## Session Goals
Build a Weather Skill
OpenWeatherMap
PyOWM Library
Python Code

---

## Build a Weather skill

* Get weather forecast for next 5 days
* Get todays weather
* Look at the types of data we can pull from Open Weather Map
* Speak the forecast on command

---

## OpenWeather Map
Where we get our weather data from
OpenWeather

* Create a free account on OpenWeather
* Create an API key (we’ll need this to access the weather data in our code)

<https://openweathermap.org/>

---

## PyOWM
Python OpenWeatherMap Library

PyOWM makes it easy to get a weather forecast
Use ‘one_call’ with lat and long and it will return a weather object containing lots of data we can use

``` python 
def weather(self):
    """ Returns the current weather at this location """
    forecast = self.mgr.one_call(lat=self.lat, lon=self.long)
    return forecast
```

#### Latitude and Longitude

Where’s you at?

* OpenWeatherMap takes a Lat and Long position
* We can get this using GeoPy - a Python Geographic Utility Library
* Geopy can take a place name and country and provide the lat and long of this location
* We can then use the Lat and Long to get the weather forecast

### Our Weather Class
What we’re aiming for

AI will say:

>Here is today’s weather: Today will be mostly <detailed_status>, with a temperature of <temperature>, humidity of <humidity>, and a pressure of <pressure> millibars. Sunrise was at  <sunrise>, and sunset is at <sunset>. The Ultaviolet level is <ultraviolet>

---

## Code