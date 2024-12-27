---
title: PCBs
description: >-
    A collection of PCBs designed by Kevin McAleer, now available for purchase.
layout: store
date: 2024-12-27
author: Kevin McAleer
mode: light
# cover: /assets/img/pcbs/cover.jpg
---

## Printed Circuit Boards

Here are a couple of PCBs I've designed for various projects. By Buying these you'll also be supporting my work and helping me to create more projects in the future.

{% include hero_left.html title="Gamepad 2.0" content="A Raspberry Pi Pico powered, progammable game-pad, for remotely controlling robots, or anything you can think of." link="#buy-one" button="Buy one" image="/assets/img/pcbs/gamepad.jpg" button_colour="ocre" bgcolor="bg-ocre" %}

{% include hero_right.html title="Servo Helper" content="Provide power to several servos motors and hookup the signal wires with a single pin, with this handy PCB" link="#buy-one" button="Buy one" image="/assets/img/pcbs/servo_helper.jpg" button_colour="ocre" bgcolor="bg-ocre" %}

{% include hero_left.html title="Picotamachibi" content="A Raspberry Pi Pico 2 powered, Virtual Pet PCB" link="#buy-one" button="Buy one" image="/assets/img/pcbs/picotamachibi.jpg" button_colour="ocre" bgcolor="bg-ocre" %}


---

## Buy One

---

<div id='collection-component-1735325804711'></div>
<script type="text/javascript">
/*<![CDATA[*/
(function () {
  var scriptURL = 'https://sdks.shopifycdn.com/buy-button/latest/buy-button-storefront.min.js';
  if (window.ShopifyBuy) {
    if (window.ShopifyBuy.UI) {
      ShopifyBuyInit();
    } else {
      loadScript();
    }
  } else {
    loadScript();
  }
  function loadScript() {
    var script = document.createElement('script');
    script.async = true;
    script.src = scriptURL;
    (document.getElementsByTagName('head')[0] || document.getElementsByTagName('body')[0]).appendChild(script);
    script.onload = ShopifyBuyInit;
  }
  function ShopifyBuyInit() {
    var client = ShopifyBuy.buildClient({
      domain: '3c2bfd-4.myshopify.com',
      storefrontAccessToken: '224a6a3a29ef2ceb1365ddc87a35b826',
    });
    ShopifyBuy.UI.onReady(client).then(function (ui) {
      ui.createComponent('collection', {
        id: '652126945624',
        node: document.getElementById('collection-component-1735325804711'),
        moneyFormat: '%C2%A3%7B%7Bamount%7D%7D',
        options: {
  "product": {
    "styles": {
      "product": {
        "@media (min-width: 601px)": {
          "max-width": "calc(25% - 20px)",
          "margin-left": "20px",
          "margin-bottom": "50px",
          "width": "calc(25% - 20px)"
        },
        "img": {
          "height": "calc(100% - 15px)",
          "position": "absolute",
          "left": "0",
          "right": "0",
          "top": "0"
        },
        "imgWrapper": {
          "padding-top": "calc(75% + 15px)",
          "position": "relative",
          "height": "0"
        }
      }
    },
    "text": {
      "button": "Add to cart"
    }
  },
  "productSet": {
    "styles": {
      "products": {
        "@media (min-width: 601px)": {
          "margin-left": "-20px"
        }
      }
    }
  },
  "modalProduct": {
    "contents": {
      "img": false,
      "imgWithCarousel": true,
      "button": false,
      "buttonWithQuantity": true
    },
    "styles": {
      "product": {
        "@media (min-width: 601px)": {
          "max-width": "100%",
          "margin-left": "0px",
          "margin-bottom": "0px"
        }
      }
    },
    "text": {
      "button": "Add to cart"
    }
  },
  "option": {},
  "cart": {
    "text": {
      "total": "Subtotal",
      "button": "Checkout"
    }
  },
  "toggle": {}
},
      });
    });
  }
})();
/*]]>*/
</script>