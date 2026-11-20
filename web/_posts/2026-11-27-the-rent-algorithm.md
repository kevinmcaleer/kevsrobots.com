---
title: "The Rent Algorithm: How Pricing Software Taught Rival Landlords to Stop Competing"
description: >-
  Millions of US apartments had their rents set with help from the same piece of software. Here's how RealPage's pricing algorithm worked, why the US Justice Department called it a cartel without a meeting room, and what the settlement changes.
excerpt: >-
  No secret meeting, no handshake, no phone call. Just rival landlords feeding their private data into the same algorithm. How rent-pricing software ended up in an antitrust case.
layout: showcase
mode: light
date: 2026-11-27
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/rent_algorithm/cover.png
hero: /assets/img/blog/rent_algorithm/cover.png
tags:
  - algorithms
  - pricing
  - housing
  - antitrust
  - machine learning
groups:
  - everyday-algorithms
---

This is the next article in a series of posts exploring how everyday algorithms influence lives, often in ways we don't immediately notice.

---

Imagine three apartment blocks on the same street, owned by three different companies. In a healthy market they compete. If one has empty flats, it drops the rent to fill them, and the others have to follow or lose tenants.

Now imagine all three quietly plug into the same piece of software. Each one hands over its private numbers: what tenants are actually paying, how many flats are empty, which leases are coming up for renewal. The software crunches everyone's data together and sends each landlord a recommended rent.

Nobody has met in a smoky back room. Nobody has shaken hands on a price. And yet the price war has stopped.

That, in a nutshell, is the case the US Department of Justice brought against a company called RealPage. It's one of the clearest examples yet of an algorithm changing how a market behaves in a way that nobody would have predicted from the code alone.

---

## What the software actually does

RealPage sells "revenue management" software to landlords, under names like YieldStar and AI Revenue Management. The idea came from the airline industry, which has used algorithms to price seats for decades.

The software does three things:

1. **Collects data.** Landlords feed in their own rents, occupancy and lease terms. According to the [DOJ's 2024 lawsuit](https://www.justice.gov/usao-mdnc/pr/justice-department-sues-realpage-algorithmic-pricing-scheme-harms-millions-american), this included "nonpublic, competitively sensitive information about their apartment rental rates and other lease terms."
2. **Models demand.** It estimates how much each unit could fetch, using everyone's data, not just yours.
3. **Recommends a price.** Every day, each landlord gets a suggested rent for each flat.

On paper a landlord can ignore the suggestion. In practice, a [2022 ProPublica investigation](https://www.propublica.org/article/yieldstar-rent-increase-realpage-rent) reported that "as many as 90%" of the software's suggestions were adopted, according to former RealPage employees. The DOJ said the company encouraged this with an "auto accept" feature and with pricing advisors "who monitor landlords' compliance."

The DOJ also said RealPage held about **80% of the market** for this kind of software in multifamily housing. When four out of five big landlords in a city are asking the same oracle, the oracle starts to look a lot like a cartel.

---

## The ratchet hidden in the rules

The most interesting part of the case isn't the data sharing. It's the shape of the algorithm's behaviour.

According to the DOJ's complaint, the software "tends to maximize price increases, minimize price decreases." Think of it as a ratchet: it turns easily one way and stiffly the other.

Here's a toy model you can run yourself to see why that matters. It isn't RealPage's code (nobody outside the company has that). It's a simple illustration of what an asymmetric pricing rule does over time.

```python
import random

random.seed(1)

def symmetric(rent, demand):
    # A normal market: rent follows demand both ways
    return rent * (1 + 0.05 * demand)

def ratchet(rent, demand):
    # Rises in full when demand is up, only a fifth as much when it's down
    step = 0.05 * demand if demand > 0 else 0.01 * demand
    return rent * (1 + step)

normal = sticky = 1000
for month in range(36):
    demand = random.uniform(-1, 1)   # demand wobbles up and down at random
    normal = symmetric(normal, demand)
    sticky = ratchet(sticky, demand)

print(f"Symmetric pricing after 3 years: ${normal:,.0f}")
print(f"Ratchet pricing after 3 years:   ${sticky:,.0f}")
```

With that random seed, the symmetric rent ends up around **$730** and the ratchet rent around **$1,210**, even though demand is pure random noise with no upward trend at all. Try other seeds: the ratchet comes out on top almost every time. A single landlord using a sticky rule like this would just lose tenants to a cheaper rival down the street. But if the rival is running the same rule, there's nobody left to undercut you.

That's the unexpected bit. **An algorithm that would fail in a competitive market can succeed when everyone uses it.**

---

## What the economists found

The best independent evidence comes from Sophie Calder-Wang and Gi Heung Kim, whose working paper [Algorithmic Pricing in Multifamily Rentals: Efficiency Gains or Price Coordination?](https://faculty.wharton.upenn.edu/?p=48581) hand-collected data on which buildings adopted the software between 2005 and 2019.

Their findings are more nuanced than the headlines, which is exactly why they're worth reading:

- **The software does make landlords more responsive.** Adopters cut rents *faster* than non-adopters during downturns. That's the "efficiency" story the software vendors tell.
- **But during recoveries, things flip.** In markets with more algorithmic pricing, average rents were higher and occupancy was lower. Higher rents *and* more empty flats is the classic fingerprint of reduced competition.
- **The coordination effect has a price tag.** Their model estimates that coordination alone adds about **$25 per unit per month**, affecting around **4.2 million units** nationwide.

The White House Council of Economic Advisers built on that research in a [2024 issue brief](https://bidenwhitehouse.archives.gov/wp-content/uploads/2024/12/Appendix-for-CEA-Issue-Brief-The-Anticompetitive-Costs-of-Algorithmic-Pricing-in-Rental-Housing.pdf), and New York's governor later cited an estimate that price-fixing algorithms cost renters [$3.8 billion in a single year](https://www.governor.ny.gov/news/governor-hochul-signs-legislative-package-bolster-homeownership-and-strengthen-protections).

ProPublica's reporting gives a more human-scale example from the prosecutors' case: one landlord "started increasing rents within a week" of adopting the software and had raised them by more than 25% within 11 months ([ProPublica, 2025](https://www.propublica.org/article/doj-realpage-settlement-rental-price-fixing-case)).

---

## So is using a pricing algorithm illegal?

No, and this is where it gets interesting for anyone who builds software.

In August 2025, the US Ninth Circuit Court of Appeals ruled in *Gibson v. Cendyn*, a case about Las Vegas hotels using a shared pricing tool. The court held that hotels independently choosing the same software, which didn't pool their confidential data and which they weren't obliged to follow, was **not** price-fixing. "Higher prices alone do not establish a violation," as [Baker McKenzie's summary](https://www.bakermckenzie.com/en/insight/publications/alerts/2025/08/united-states-ninth-circuit-on-algorithmic-pricing) puts it.

So the legal line isn't "algorithms bad". It's much more specific:

| Probably fine | The danger zone |
|---|---|
| Your algorithm learns from **your own** data | It learns from **rivals'** private data |
| It uses **public** prices (listings anyone can see) | It uses **non-public** prices (real lease contracts) |
| You can ignore it with no consequences | Advisors check you're following it |
| It moves prices up and down evenly | It has built-in limits on cutting prices |

That table is, more or less, the shape of the settlement.

---

## What the settlement changes

In November 2025 the DOJ [announced a proposed settlement](https://www.justice.gov/opa/pr/justice-department-requires-realpage-end-sharing-competitively-sensitive-information-and). If the court approves it, RealPage must:

- Stop its software using competitors' non-public data to set prices "in runtime operation".
- Stop training its models on active lease data. Training data must be **at least 12 months old**.
- "Remove or redesign features that limited price decreases or aligned pricing between competing users". In other words, take the ratchet out.
- Accept a court-appointed monitor.

RealPage said the settlement involves no financial penalty and no admission of wrongdoing. The DOJ's separate case against several large landlords carries on. The largest, Greystar, settled in August 2025, again without admitting wrongdoing.

Here's the clever bit that I haven't seen discussed much. The 12-month rule turns the shared dataset from a **live feed** into a **history book**. Year-old data is still useful for learning seasonal patterns ("rents rise in summer"), but it's useless for coordinating today's prices ("your rival just put their rent up, so you can too"). It's a neat engineering fix to a legal problem: the regulators didn't ban the machine learning, they changed what time the model is allowed to see.

---

## The states go further

Some lawmakers decided that line-drawing wasn't enough. In October 2025 New York became, according to [Wilson Sonsini](https://www.wsgr.com/en/insights/new-york-prohibits-use-of-pricing-algorithms-for-rent-setting.html), "the first state to outright ban the use of pricing algorithms by landlords" that pool data from two or more owners to recommend rents. California passed its own law the same month.

---

## Why this matters beyond rent

The RealPage story is a preview of a problem that's going to keep coming up as more businesses hand pricing over to software:

- **Algorithms don't need to talk to each other to coordinate.** Sharing a model, or sharing training data, can be enough.
- **Small design choices compound.** A rule that cuts prices a little slower than it raises them looks harmless in a single month and adds up across millions of homes.
- **Where the data comes from matters as much as what the code does.** The same algorithm trained on your own data is a tool; trained on your competitors' live data, it's something else.

If you're building anything that learns from data, from a robot that adapts its behaviour to a model that sets prices, it's worth asking the RealPage question: *whose data is this, and what happens when everyone uses the same model?*

---

## Sources

- US Department of Justice, [Justice Department Sues RealPage for Algorithmic Pricing Scheme that Harms Millions of American Renters](https://www.justice.gov/usao-mdnc/pr/justice-department-sues-realpage-algorithmic-pricing-scheme-harms-millions-american) (23 August 2024)
- US Department of Justice, [Justice Department Requires RealPage to End the Sharing of Competitively Sensitive Information](https://www.justice.gov/opa/pr/justice-department-requires-realpage-end-sharing-competitively-sensitive-information-and) (24 November 2025)
- Heather Vogell, ProPublica, [Rent Going Up? One Company's Algorithm Could Be Why](https://www.propublica.org/article/yieldstar-rent-increase-realpage-rent) (October 2022)
- ProPublica, [DOJ reaches settlement with RealPage in rental price-fixing case](https://www.propublica.org/article/doj-realpage-settlement-rental-price-fixing-case) (November 2025)
- Sophie Calder-Wang and Gi Heung Kim, [Algorithmic Pricing in Multifamily Rentals: Efficiency Gains or Price Coordination?](https://faculty.wharton.upenn.edu/?p=48581) (working paper)
- Council of Economic Advisers, [Appendix: The Cost of Anticompetitive Pricing Algorithms in Rental Housing](https://bidenwhitehouse.archives.gov/wp-content/uploads/2024/12/Appendix-for-CEA-Issue-Brief-The-Anticompetitive-Costs-of-Algorithmic-Pricing-in-Rental-Housing.pdf) (December 2024)
- Baker McKenzie, [Ninth Circuit on Algorithmic Pricing](https://www.bakermckenzie.com/en/insight/publications/alerts/2025/08/united-states-ninth-circuit-on-algorithmic-pricing) (August 2025)
- New York State Governor's Office, [Governor Hochul Signs Legislative Package](https://www.governor.ny.gov/news/governor-hochul-signs-legislative-package-bolster-homeownership-and-strengthen-protections) (16 October 2025)
- Wilson Sonsini, [New York Prohibits Use of Pricing Algorithms for Rent-Setting](https://www.wsgr.com/en/insights/new-york-prohibits-use-of-pricing-algorithms-for-rent-setting.html) (October 2025)
