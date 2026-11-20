---
title: "The Surge Engine: What Dynamic Pricing Algorithms Really Do (and What They Get Blamed For)"
description: >-
  Surge pricing is the algorithm everyone loves to hate. Uber's own data shows what happens when it switches off, round numbers change how riders behave, and the UK regulator found the Oasis ticket fiasco wasn't an algorithm at all.
excerpt: >-
  When Uber's surge pricing broke on New Year's Eve, waits tripled. When Oasis tickets doubled in price, everyone blamed an algorithm that, according to the regulator, wasn't there. The strange psychology of prices that move.
layout: showcase
mode: light
date: 2026-12-11
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/surge_engine/cover.png
hero: /assets/img/blog/surge_engine/cover.png
tags:
  - algorithms
  - pricing
  - surge pricing
  - behavioural economics
  - dark patterns
groups:
  - everyday-algorithms
---

It's raining, it's late, and you open a ride-hailing app. The price is **2.1×** normal. You sigh, and you book it anyway.

Few algorithms make people as angry as surge pricing. It feels like the app is kicking you while you're down. But the story of what surge pricing actually does, and what it gets blamed for, is far stranger than "big tech squeezes customers". It involves a broken server on New Year's Eve, the curious power of round numbers, and a ticket sale where the algorithm everyone blamed turned out, according to the regulator, not to exist.

---

## What the surge algorithm actually does

Uber's version is surprisingly simple at heart. According to a [2017 NBER working paper](https://www.nber.org/system/files/working_papers/w23296/revisions/w23296.rev0.pdf) co-written by Uber's then head of economic research, Keith Chen, the base fare is "dynamically updated in each 'geofence' neighborhood at five minute intervals by a 'surge' multiplier."

So the city is split into zones, and every five minutes the algorithm asks each zone a question: *are there more people wanting rides than drivers available?* If so, the multiplier goes up.

A higher price is meant to do two jobs at once:

1. **Pull in drivers.** A 2× fare is a reason for a driver to log on, or to drive across town to where the riders are.
2. **Sort the riders.** Some people decide to walk, wait or take the bus, leaving the cars for the people who most need them right now.

That's the theory. The interesting question is whether it works, and that's where an accident comes in.

---

## The night surge pricing broke

Uber's own economists, Jonathan Hall, Cory Kendrick and Chris Nosko, published a case study called [The Effects of Uber's Surge Pricing](https://www.uber.com/blog/research/the-effects-of-ubers-surge-pricing-a-case-study) built around two nights in New York.

**Night one: a sold-out concert at Madison Square Garden.** As the crowd poured out, demand spiked and surge kicked in. Prices went up, more drivers headed for the area, and the average wait stayed at around **2.6 minutes**.

**Night two: New Year's Eve, when surge pricing went down.** A technical fault meant the multiplier stopped working during one of the busiest nights of the year. [TechCrunch reported](https://techcrunch.com/2015/09/17/ubers-latest-justification-for-surge-pricing/amp/) the outage lasted 26 minutes. During it, waiting times spiked to **8 minutes** and the share of ride requests that went unfulfilled rose to **25%**.

Without the price signal, rides didn't get cheaper for everyone. They simply became *unavailable* for a quarter of the people asking, and they went to whoever happened to tap first rather than whoever needed them most.

Two health warnings. First, this is Uber's own research about Uber's own product, so treat it like a company's homework marked by the company. Second, TechCrunch noted a separate Washington Post analysis suggesting surge mostly *relocates* drivers already on the road rather than summoning new ones. Both can be true: surge can be good at moving cars to where they're needed without making the total pool much bigger.

---

## The nudge in the number

Here's where it gets psychological.

In 2016, Keith Chen talked to NPR's [Hidden Brain](https://www.npr.org/2016/05/17/478266839/this-is-your-brain-on-uber) podcast about what Uber had learned from watching millions of riders decide whether to accept a surge price. One finding is a gem: riders were **more** likely to accept a **2.1×** surge than a **2.0×** one.

Read that again. A *higher* price was accepted more readily than a lower one.

A plausible explanation is about how humans read numbers. A clean "2.0×" looks like a deliberate, human decision: someone has *chosen* to double your fare. "2.1×" looks like the output of a machine doing sums. It feels less personal and more like the weather, and we resent the weather less than we resent being gouged.

Chen also revealed, according to [Engadget's write-up](https://www.engadget.com/2016-05-20-uber-surge-pricing-battery-life.html), that people whose phone battery is about to die are more willing to pay a surge price. The app knows your battery level (it uses it to switch into a power-saving mode). Chen was clear that Uber does **not** use battery level to set prices. But think about what that finding means: the company had the data to show that a desperate customer is a less price-sensitive customer, and the only thing standing between that insight and a pricing feature was a decision not to build it.

That's the real lesson of algorithmic pricing. **The algorithm learns what you'll tolerate. Policy decides what it's allowed to act on.**

---

## The Oasis plot twist

Now for the most famous "dynamic pricing" story in recent British memory.

In August 2024, millions of fans queued online for tickets to the Oasis reunion tour. Many reached the front of the queue only to find that standing tickets were suddenly far more expensive than they'd expected. Social media exploded with anger at Ticketmaster's "dynamic pricing algorithm". The UK's Competition and Markets Authority (CMA) [opened an investigation](https://www.gov.uk/cma-cases/ticketmaster-consumer-protection-case) in September 2024.

When the CMA [published its findings](https://www.gov.uk/government/news/cma-secures-changes-from-ticketmaster-following-oasis-tickets-investigation) in September 2025, it contained a twist almost nobody reported in the headlines: the CMA **"has not found evidence"** that Ticketmaster used algorithmic dynamic pricing for the Oasis sale.

What it found instead was older and simpler:

- Standing tickets were sold at **two fixed price tiers**. When the cheaper tier sold out, everyone behind them in the queue got the expensive one.
- Fans in the queue **weren't told** this would happen.
- Some "platinum" tickets were sold at **almost 2.5 times** the price of standard tickets, without a clear explanation that they offered no extra benefits.

So the villain wasn't a machine learning model watching demand in real time. It was a human-designed price list combined with a queue that hid information. It *felt* algorithmic because the price changed while people waited, and in 2024 "the algorithm did it" was the obvious explanation.

Ticketmaster gave the CMA undertakings, "without any admission of wrongdoing or liability", to warn fans 24 hours in advance when tiered pricing is used, show the price range when people join a queue, update them when cheaper tickets sell out, drop misleading labels, and report to the CMA for two years. In Ireland, the [Irish Times reported](https://www.irishtimes.com/culture/music/2025/09/25/ticketmaster-commits-to-greater-price-transparency-after-oasis-sale/) that the Competition and Consumer Protection Commission is still investigating the Croke Park sale.

---

## What makes a moving price feel fair?

Put the Uber and Oasis stories side by side and a pattern appears. People don't object to prices changing. Theatres, trains and hotels have always charged more at busy times. What people object to is:

| Feels fair | Feels like a trap |
|---|---|
| You see the price **before** you commit | The price changes **after** you've queued for hours |
| The reason is obvious (rain, rush hour, New Year's Eve) | The reason is hidden |
| The same price applies to everyone in that moment | It might depend on something about **you**, like your battery |
| A number that looks computed (2.1×) | A number that looks chosen (2.0×) |

That last row is the uncomfortable one. It means that how a price is *displayed* can change how fair it feels, without the price itself changing at all. That's a nudge, and companies know it works.

---

## The takeaway

Surge pricing is a neat bit of engineering: a feedback loop that uses price as a signal to balance supply and demand every five minutes, a bit like a thermostat for taxis. When it switched off on New Year's Eve, the system didn't become fairer; it just broke.

But the same data that makes the loop work also reveals exactly how desperate, rushed or low-on-battery each of us is. The algorithm doesn't need to be evil to be worrying. It just needs to be good at learning, with nobody deciding what it's allowed to learn *from*.

And the next time a price jumps while you're waiting and everyone blames "the algorithm"? Check first. Sometimes it's a machine. Sometimes, as Oasis fans found out, it's just a spreadsheet and a queue.

---

## Sources

- Jonathan Hall, Cory Kendrick and Chris Nosko, [The Effects of Uber's Surge Pricing: A Case Study](https://www.uber.com/blog/research/the-effects-of-ubers-surge-pricing-a-case-study) (Uber, 2015)
- Megan Rose Dickey, TechCrunch, [Uber's Latest Justification For Surge Pricing](https://techcrunch.com/2015/09/17/ubers-latest-justification-for-surge-pricing/amp/) (17 September 2015)
- M. Keith Chen, Judith Chevalier, Peter Rossi and Emily Oehlsen, [The Value of Flexible Work: Evidence from Uber Drivers](https://www.nber.org/system/files/working_papers/w23296/revisions/w23296.rev0.pdf), NBER Working Paper 23296 (2017)
- NPR Hidden Brain, [This Is Your Brain On Uber](https://www.npr.org/2016/05/17/478266839/this-is-your-brain-on-uber) (May 2016)
- Engadget, [Uber says you're willing to pay more when your phone is dying](https://www.engadget.com/2016-05-20-uber-surge-pricing-battery-life.html) (20 May 2016)
- Competition and Markets Authority, [CMA secures changes from Ticketmaster following Oasis tickets investigation](https://www.gov.uk/government/news/cma-secures-changes-from-ticketmaster-following-oasis-tickets-investigation) (25 September 2025)
- Competition and Markets Authority, [Ticketmaster: consumer protection case](https://www.gov.uk/cma-cases/ticketmaster-consumer-protection-case)
- Conor Pope, Irish Times, [Ticketmaster commits to greater price transparency after Oasis sale](https://www.irishtimes.com/culture/music/2025/09/25/ticketmaster-commits-to-greater-price-transparency-after-oasis-sale/) (25 September 2025)
