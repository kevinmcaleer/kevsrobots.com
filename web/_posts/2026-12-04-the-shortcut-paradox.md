---
title: "The Shortcut Paradox: Why Your Sat-Nav Can Make Everyone's Journey Slower"
description: >-
  Every driver following their own fastest route sounds efficient. Maths from 1968 says it can make everyone slower. How navigation apps turn a game theory puzzle into real traffic, from Los Angeles back streets to a New Jersey town that shut its roads.
excerpt: >-
  Add a new road and traffic gets worse. Remove one and it gets better. The 1968 maths puzzle that explains what happens when millions of drivers all follow their sat-nav.
layout: article
hero_style: split
mode: light
date: 2026-12-04
author: Kevin McAleer
difficulty: beginner
specs:
  Drivers: 4,000
  Without the shortcut: 65 min
  With the shortcut: 80 min
cover: /assets/img/blog/shortcut_paradox/cover.png
hero: /assets/img/blog/shortcut_paradox/cover.png
tags:
  - algorithms
  - navigation
  - game theory
  - path planning
  - traffic
groups:
  - everyday-algorithms
---

This is the next article in a series of posts exploring how **everyday algorithms** influence lives, often in ways we don't immediately notice.

---

Your sat-nav has one job: get *you* there as fast as possible. It looks at live traffic, runs a shortest-path search (the same family of algorithms we use to steer robots around a room) and picks the quickest route.

That sounds like it can only make things better. If everyone gets the fastest route, surely everyone gets there faster?

Not quite. In 1968 a German mathematician called Dietrich Braess showed that when every driver picks their own best route, **adding a road can make every single journey slower**. For decades it was a curiosity in transport textbooks. Then we put a route-optimising computer in every car and every phone, and the curiosity moved onto real streets.

## The puzzle: 4,000 drivers and a shortcut

Here's the classic version of [Braess's paradox](https://en.wikipedia.org/wiki/Braess%27s_paradox), the one shown on the cover diagram.

4,000 drivers want to get from **Start** to **End**. There are two routes:

- **Top route:** Start → A takes *(number of cars ÷ 100)* minutes, because it's a narrow road that clogs up. Then A → End is a big motorway that always takes 45 minutes.
- **Bottom route:** Start → B is the motorway (45 minutes), then B → End is the narrow road (*cars ÷ 100* minutes).

The two routes are mirror images, so drivers split evenly: 2,000 each way. Every driver's journey takes 2,000 ÷ 100 + 45 = **65 minutes**.

Now the council builds a brilliant new shortcut from A to B that takes almost no time at all. Watch what happens.

A driver on the top route looks at their sat-nav. Start → A is 20 minutes. Instead of then taking the 45-minute motorway, they can nip across the shortcut and take the narrow road B → End, which is also only 20 minutes. So their sat-nav reroutes them. And the next driver. And the next.

Once everyone has worked this out, all 4,000 drivers go Start → A → B → End. Both narrow roads now carry 4,000 cars: 40 minutes each. The total is **80 minutes**.

Every single driver is 15 minutes slower than before the shortcut existed. And here's the cruel part: nobody can fix it on their own. If you switch back to the old route, your first leg is still 40 minutes (everyone else is on it) plus 45 on the motorway: 85 minutes. Staying on the shortcut is still your best choice. Everyone is stuck.

<figure class="text-center my-4">
  <img src="/assets/img/blog/shortcut_paradox/braess-before-after.png" alt="Two road networks: before the shortcut 2,000 cars take each route and every trip takes 65 minutes; after it all 4,000 cars take the shortcut and every trip takes 80 minutes" loading="lazy" class="img-fluid rounded-3">
  <figcaption class="text-muted small mt-2">Same drivers, same destination, one extra road: every trip gets 15 minutes longer</figcaption>
</figure>

{% callout note "Nash equilibrium" %}
Game theorists call this a **Nash equilibrium**: a situation where nobody can do better by changing only their own choice, even though everyone could do better if they all changed together.
{% endcallout %}

## Try it yourself

{% callout try "Try it yourself" %}
You can simulate this with a few lines of Python. Each "driver" in turn looks at the current traffic and picks the fastest route, exactly like a sat-nav does.
{% endcallout %}

```python
DRIVERS = 4000

def journey_times(top, bottom, shortcut):
    start_a = (top + shortcut) / 100        # narrow road, shared by top + shortcut
    b_end = (bottom + shortcut) / 100       # narrow road, shared by bottom + shortcut
    return {
        "top": start_a + 45,
        "bottom": 45 + b_end,
        "shortcut": start_a + 0 + b_end,
    }

def simulate(shortcut_open):
    counts = {"top": 0, "bottom": 0, "shortcut": 0}
    for _ in range(DRIVERS):
        # each new driver asks "what's fastest if I join now?"
        options = ["top", "bottom"] + (["shortcut"] if shortcut_open else [])
        best = min(options, key=lambda r: journey_times(
            **{k: counts[k] + (1 if k == r else 0) for k in counts})[r])
        counts[best] += 1
    times = journey_times(**counts)
    return counts, max(times[r] for r in counts if counts[r] > 0)

for road in (False, True):
    counts, worst = simulate(road)
    print(f"Shortcut open: {road}  routes: {counts}  journey: {worst:.0f} min")
```

Which prints:

```text
Shortcut open: False  routes: {'top': 2000, 'bottom': 2000, 'shortcut': 0}  journey: 65 min
Shortcut open: True  routes: {'top': 0, 'bottom': 0, 'shortcut': 4000}  journey: 80 min
```

{% callout tip "The paradox in one line" %}
Each driver made the smart choice. The crowd made the dumb one.
{% endcallout %}

## It isn't just a textbook trick

The [Wikipedia article on the paradox](https://en.wikipedia.org/wiki/Braess%27s_paradox) collects real cases where the reverse happened, and *removing* a road sped traffic up:

- **Stuttgart, 1969:** after new roads were built, traffic didn't improve until a section of new road was closed again.
- **New York, 1990:** closing 42nd Street for Earth Day *reduced* congestion in the area.
- **Seoul, 2003–2005:** traffic around the city sped up after the Cheonggye Expressway was demolished and replaced with a park and stream.

How bad can it get? In a famous paper, [How Bad Is Selfish Routing?](https://www.timroughgarden.org/papers/routing.pdf), computer scientists Tim Roughgarden and Éva Tardos proved that on networks where delays grow in a straight line with traffic, everyone choosing selfishly gives total journey times **at most 4/3 of the best possible**, or 33% worse. That's a worst case. In our example, a traffic controller who could tell drivers which way to go would send just 500 of them over the shortcut, for an average trip of about 64.7 minutes. Selfish routing gives 80, about 24% worse. This gap between what selfish drivers get and what a coordinator could achieve has a wonderful name: the **price of anarchy**.

## Enter the apps

Before sat-navs, most drivers stuck to main roads because they didn't know the back streets. Their ignorance accidentally kept them closer to the system optimum. Navigation apps changed that. They know every rat-run, they react in seconds, and they're used by millions of people at once.

Researchers at UC Berkeley's Institute of Transportation Studies, led by Théophile Cabannes and Alexandre Bayen, studied exactly this in [The Impact of GPS-Enabled Shortest Path Routing on Mobility](https://its.berkeley.edu/publications/impact-gps-enabled-shortest-path-routing-mobility-game-theoretic-approach) (presented at the Transportation Research Board in 2018). Looking at the I-210 corridor in the Los Angeles area, [their paper](https://bayen.berkeley.edu/sites/default/files/cabannes.pdf) found:

- At peak times, side-road detours could be **as much as 20% faster** than staying on the freeway.
- Over four years, traffic leaving the freeway at two exits grew **1.5-fold and 3-fold**.
- In one Sherman Oaks example they cite, a residential street hit **more than 1000%** of its sustainable load at peak hour.

[NPR reported](https://www.npr.org/sections/alltechconsidered/2016/06/13/481871496/the-road-less-traveled-not-since-waze-came-to-los-angeles) on the same Sherman Oaks streets, where residents blamed Waze for morning queues of "thirty cars already just waiting" by 7 a.m. To be fair, the city's transport department pointed out that cut-through traffic "is definitely not new".

## The insight most articles miss: the advantage evaporates

Here's the part of the Berkeley research I find most interesting, and it's buried in a table.

The team simulated a stretch of the I-210 with different shares of drivers using navigation apps. When **25%** of drivers used an app, app users' journeys were about **48% shorter** than everyone else's. At **50%**, the advantage shrank to about **36%**. At **75%**, it was down to about **27%**.

In other words, a navigation app is most valuable when *other people aren't using it*. Its edge comes from knowing a shortcut the crowd doesn't. As more drivers get the same advice, the shortcut fills up and the advantage drains away, slowly heading towards the Braess endpoint where everyone has the same "smart" route and nobody is better off.

<figure class="text-center my-4">
  <img src="/assets/img/blog/shortcut_paradox/app-advantage.png" alt="Bar chart: when 25% of drivers use a navigation app they save 48% on travel time, at 50% they save 36%, at 75% they save 27%" loading="lazy" class="img-fluid rounded-3">
  <figcaption class="text-muted small mt-2">The app's edge shrinks as more drivers get the same advice</figcaption>
</figure>

{% callout warning "What the study doesn't say" %}
The authors are careful here: they say that in their model the rerouting "seems to make the system more efficient" overall, and they stress that their measurements "confirm phenomena and trends, but do not prove them". So this isn't proof that apps make traffic worse in general. It's evidence that the benefit is a relative one, and that the cost lands on people who aren't using the app at all: the residents of the side streets.
{% endcallout %}

## When a town fights back

Leonia, a small New Jersey town next to the George Washington Bridge into Manhattan, became the poster child for this problem. Officials said Waze and Google Maps were sending bridge traffic through its residential streets whenever the main roads backed up.

In January 2018 the town [closed its local streets to non-residents](https://abc7ny.com/post/nj-town-to-close-streets-to-non-residents-during-rush-hour-/2974234/) during rush hours, with a **$200 fine** for drivers who weren't residents. According to a [CityLab report summarised by Planetizen](https://www.planetizen.com/news/2018/04/98338-roads-residents-only), the ban covered **60 public roads**. It did calm the streets, but local businesses complained about losing customers.

It didn't last. In August 2018 a state judge [struck the closures down](https://www.fox5ny.com/news/judge-tosses-leonias-ban-on-nonresident-drivers), after a challenge from a driver who lived in a nearby town and from New Jersey's Department of Transportation. The ruling turned on a technicality with a big idea behind it: the closures affected a state highway, so the town needed the state's permission, which it never got. A road network is shared, and one town can't opt out of it, even when an algorithm is the one sending people through.

Leonia's story shows the real problem. The app is solving the right problem for each user and the wrong problem for the town. There's nobody whose job it is to optimise the whole network.

## What robots can teach sat-navs

If you've ever built a robot that plans a path, you've used the same algorithms your sat-nav uses: Dijkstra's algorithm or A*. Each robot finds the shortest path from where it is to where it wants to be.

But look at a warehouse full of hundreds of delivery robots and you'll notice they *don't* each pick their own selfish shortest path. If they did, they'd all pile into the same aisle and gridlock. Instead, fleets are usually coordinated centrally: a planner looks at the whole warehouse and hands out routes that are good for the *system*, even if one robot sometimes gets a slightly longer trip.

That's the difference between a **user equilibrium** (everyone does what's best for them) and a **system optimum** (do what's best for everyone). Robot fleets can choose the second because one company owns all the robots. Roads can't, because every driver is their own boss.

<figure class="text-center my-4">
  <img src="/assets/img/blog/shortcut_paradox/fleet.png" alt="Two warehouse floor plans: on the left every robot queues in the same aisle; on the right a central planner spreads them across all the aisles" loading="lazy" class="img-fluid rounded-3">
  <figcaption class="text-muted small mt-2">Selfish shortest paths jam one aisle; a fleet planner spreads the load</figcaption>
</figure>

{% fullwidth style="dark" %}
## The Bottom Line

So the next time your sat-nav sends you on a wild detour through a housing estate to save ninety seconds, remember: it's playing a game against every other driver's sat-nav. And as Braess showed back in 1968, sometimes everyone playing that game perfectly is exactly how everyone loses.
{% endfullwidth %}

## Sources

- Dietrich Braess, [Über ein Paradoxon aus der Verkehrsplanung](https://doi.org/10.1007/bf01918335), *Unternehmensforschung* 12, 258–268 (1968)
- Wikipedia, [Braess's paradox](https://en.wikipedia.org/wiki/Braess%27s_paradox), for the worked 4,000-driver example and the Stuttgart, New York and Seoul cases (each cited there to its original source)
- Tim Roughgarden and Éva Tardos, [How Bad Is Selfish Routing?](https://www.timroughgarden.org/papers/routing.pdf), *Journal of the ACM* (preliminary version at FOCS 2000)
- Théophile Cabannes, Alexandre Bayen et al., [The Impact of GPS-Enabled Shortest Path Routing on Mobility: A Game Theoretic Approach](https://its.berkeley.edu/publications/impact-gps-enabled-shortest-path-routing-mobility-game-theoretic-approach), Transportation Research Board 97th Annual Meeting (2018). [Full paper (PDF)](https://bayen.berkeley.edu/sites/default/files/cabannes.pdf)
- NPR, [The Road Less Traveled? Not Since Waze Came To Los Angeles](https://www.npr.org/sections/alltechconsidered/2016/06/13/481871496/the-road-less-traveled-not-since-waze-came-to-los-angeles) (June 2016)
- ABC7 New York, [New Jersey town to close streets to non-residents during rush hour](https://abc7ny.com/post/nj-town-to-close-streets-to-non-residents-during-rush-hour-/2974234/) (January 2018)
- Planetizen, [Roads for Residents Only](https://www.planetizen.com/news/2018/04/98338-roads-residents-only) (April 2018)
- FOX 5 New York, [Judge tosses Leonia's ban on nonresident drivers](https://www.fox5ny.com/news/judge-tosses-leonias-ban-on-nonresident-drivers) (August 2018)
