# Off-board sources of road surface condition and friction ahead (maps, road weather, connected vehicles)

Scope: what a planner can learn from outside the car about each segment of the 200-260 m window
ahead (surface class: peak friction and fall-off; corner geometry, grade, width), as input for
roadmap 061 M3b (seeds by precondition: dry / wet mu ~0.6-0.8 / low mu <= 0.45; hairpin >= 150 deg,
R <= 25 m; downhill > 5 %; width arm 6-15 m). Grounding: `docs/roadmap/061-grip-or-drift-mode-choice.md`
(M3b table; "the surface ahead is the map's; an unseen patch (060 M6) stays outside"); the course
generator's `data/courses/haruna/surfaces.yaml` today holds only label defaults
(`Road: asphalt, Terrain: grass, SideWalk: concrete, Ground: dirt`, `zones: []`).

Research budget note: ~22 tool calls; several vendor PDFs were binary-only and were text-extracted
locally. Anything not traced to a fetched source is in Gaps or marked "unverified".

## 1. Road Weather Information Systems (RWIS / ESS stations)

### Takeaway
RWIS stations measure surface state (dry/wet/ice/snow), surface temperature and, with optical
sensors, a "grip" index, but they are point sensors kilometres apart (Sweden ~775 stations,
Finland >350, US >2,000 ESS c. 2008) updated every 1-30 min; their grip index correlates poorly with
measured friction at low mu. For a 200-260 m window they give a regional prior, not a per-segment value.

### Cited Findings
- Sweden (Trafikverket VViS): data from ~775 measuring stations along the road network (2019);
  30-min averages for wind, precipitation type and amount; air/surface/dew point temperatures are
  instantaneous readings 30 s before collection times — [Trafikverket ersättningsmodell väder 2019](https://bransch.trafikverket.se/contentassets/28744f2661f44c22b63b4042092fadfd/ersattningsmodell_vaders_2019.pdf); same 775 figure in [Trafikverket/SMHI 2019 presentation](https://www.smhi.se/download/18.2921e83919a90b6870284e/1763456882496/Metodkonferensen%202019%20-%20Jonas%20Jonsson%20-%20Trafikverkets%20n%C3%A4sta%20generations%20v%C3%A4gv%C3%A4derinformationssystem.pdf).
- Sweden, older figure: ~800 stations, updates every 30 min in winter with a planned move to 10 min;
  only ~370 report in summer (2014 DATEX II presentation) — [Bjerkholt, Weather data Sweden](https://akce.fd.cvut.cz/sites/default/files/datex2/presentations/D2_05a_02_Bo_Bjerkholt_Weather_data_Sweden.pdf).
- Sweden replacement: Vaisala framework agreement (2019) for a minimum of 600 new road weather stations — [Vaisala press release](https://vaisala.com/press-releases/2019-02/vaisala-provide-swedish-transport-administration-new-road-weather-station-network).
- Finland (Fintraffic Digitraffic): "over 350 road weather stations on the Finnish road network";
  data updated once per minute; REST/JSON, MQTT over WebSockets for real-time, DATEX II XML for
  traffic messages — [Digitraffic road traffic](https://www.digitraffic.fi/en/road-traffic/). (The page lists temperature, wind, rain, humidity, dew point; whether surface-state/friction sensors are in the public sensor list was not confirmed.)
- Norway publishes road weather data via DATEX II — [Statens vegvesen open data](https://www.vegvesen.no/en/about-us/about-us/open-data/datex/publications/road-weather-data/) (content not fetched; existence only).
- USA: "more than 2,100 ESSs are currently in use" (archived FHWA Clarus overview, c. 2008); most of
  them (>2,000) are RWIS field components — [Clarus overview (archived)](https://webarchive.library.unt.edu/eot2008/20081105043300/http:/www.its.dot.gov/clarus/clarus_overview.htm); [FHWA Road Weather Management](https://ops.fhwa.dot.gov/weather/mitigating_impacts/surveillance.htm).
- Clarus functionality was transitioned into NOAA MADIS, with an RWIS data category scheduled by
  Q3 FY2016; coverage CONUS, Alaska, Canada — [MADIS RWIS/Clarus](https://madis.ncep.noaa.gov/madis_rwis_clarus.shtml); [MADIS RWIS](https://madis.ncep.noaa.gov/madis_rwis.shtml).
- Station density matters for estimation quality between stations (Iowa optimal RWIS density
  study) — [Iowa DOT, Optimal RWIS sensor density phase III](https://publications.iowa.gov/39916/2/optimal_RWIS_sensor_density_and_location_phase-III_t2.pdf). No spacing number extracted.
- Non-intrusive optical sensor (Vaisala DSC111 class): infrared beam aimed at the road, nominal
  sensitivity 10 µm water/ice; Ontario field test: surface-state match rate >85 %; Montana study:
  grip levels did not correlate well with friction-meter coefficients, especially at low friction;
  Finnish Utti test: modelled grip vs measured friction RMS difference 0.07 — as summarised from
  [Montana State (scholarworks)](https://scholarworks.montana.edu/server/api/core/bitstreams/bd414ee0-f7a4-49b7-b73d-580fb18b6539/content), [TRB Weather 2008 non-intrusive sensors](https://fluid.yodify.com/Documentation/Document/eevPW/RDS-TRB-Weather2008-002-NonIntrusive-road-weather-sensors.pdf), [Roads & Bridges 2013](https://roadsandbridges.s3.amazonaws.com/s3fs-public/44_non-invasive%20road%20sensors_RandB0313.pdf). (Numbers from search summaries; the individual papers were not opened — verify before quoting.)
- NIRA's own SIRWEC 2022 slide illustrates the spatial problem: a camera image and the temperature
  RWIS used for the same spot were 7 km apart — [NIRA, SIRWEC 2022 (Zachrisson Hägg)](https://sirwec.org/wp-content/uploads/2023/10/sirwec2022_5.2_presentation_Zachrisson_Hagg.pdf).
- US RWIS standards (NTCIP 1204 ESS, siting guidelines) give spacing/siting guidance — [USDOT RWIS standards](https://www.standards.its.dot.gov/Content/documents/rwis-standards.htm).

### Inferences
- With stations every several to tens of km, RWIS cannot resolve a 200-260 m window. Use it as a
  regional class prior (dry / wet / near-freezing / icy) and as a trigger: surface temperature
  <= ~0 degC with moisture -> enable the "low friction" seed row of M3b everywhere ahead.
- RWIS "grip" (0-1 index) is not tyre peak mu; map it only to a coarse class with a wide band.
- For a race on a closed course the analogue is a trackside weather station plus surface
  thermometers; same role (prior, not per-segment truth).

### Gaps
- Japan (MLIT / NEXCO road weather telemeters) and Clarus-era current US counts and update
  intervals: no source fetched. MADIS update schedule not found.
- Station spacing rules (FHWA/Aurora siting guidelines) not extracted.

## 2. Connected-vehicle friction / slippery-road data

### Takeaway
Commercial floating-car friction (NIRA, Volvo/Data for Road Safety, HERE, TomTom, Continental,
RoadCloud, Klimator) delivers event-level "slippery road" alerts or segment grip classes with
seconds-to-minutes latency and ~10 m map matching (claimed), from fleets of 10^5-10^6 cars. Public
accuracy evidence is thin and mostly vendor-sourced; only probe-style dedicated sensors (Teconer)
have published error figures (~0.1 mu).

### Cited Findings
- NIRA Dynamics: edge software in production cars estimates available road friction from wheel
  speed/chassis signals; measured data types: available road friction, ambient temperature, wet
  state, road surface temperature; "about 100 000 vehicles" contributing measurements on one day in
  the UK; products: slippery road, slippery-when-wet, road state, potholes, wetness, surface temperature;
  "Mobile RWIS" fuses car data with road weather models (road condition, grip, ambient, modelled RST);
  regional "slipperiness alerts" based on multiple car measurements; an example where slipperiness
  was observed 1 h 40 min before accidents — [NIRA, SIRWEC 2022](https://sirwec.org/wp-content/uploads/2023/10/sirwec2022_5.2_presentation_Zachrisson_Hagg.pdf).
- NIRA Road Surface Alerts with Audi/HERE: aggregated vehicle data plus weather metadata, sent to
  HERE; "near real time"; no latency figure — [NIRA/Audi press release](https://niradynamics.se/precise-data-for-greater-safety-nira-dynamics-launches-road-surface-alerts-with-audi-to-improve-slippery-roads-warning-system/); [NIRA Road Surface Alerts](https://niradynamics.se/road-surface-alerts/).
- Bosch integrates NIRA alerts; map-matched to <10 m precision; Europe and North America first,
  fleet/insurer availability from 2026 — [motorsactu (French)](https://motorsactu.com/securite-routiere-bosch-integre-les-road-surface-alerts-de-nira-dynamics-dans-ses-services-connectes/). Unverified against NIRA primary pages.
- VW Group: around three million vehicles can contribute grip data to NIRA swarm friction maps — [CARIAD, Getting a grip on road conditions](https://cariad.technology/de/en/news/stories/getting-the-grip-on-road-conditions.html); [VW Group Italia](https://www.volkswagengroup.it/en/mobotics/increasingly-accurate-maps-for-safe-driving-even-on-slippery-roads).
- Public accuracy of NIRA estimates: per Langstand & Rabi (2023), the only public information is a
  bachelor's thesis comparing against a friction wheel on snowy/icy Swedish roads (21 runs); their
  own simulated cooperative estimator reaches worst-case consensus error ~10 % with >= 50 recently
  passed vehicles, regressor RMS < 5 %, and claims half the RMS of "a commercial friction estimation
  service" — [arXiv:2302.03560](https://arxiv.org/abs/2302.03560) (simulation, submitted to CCTA 2023).
- Validation methodology: reference friction for validating estimators from Swedish winter-monitoring
  trailer equipment mapped to car-tyre friction (Hjort, Bruzelius, Kharrazi, Yang; VTI/Chalmers/Volvo,
  AVEC 2024) — [Chalmers research 543381](https://research.chalmers.se/en/publication/543381).
- US evaluation project with NIRA as performer, Iowa State CTRE managing (2023-2024), results not
  listed — [TRB RiP 2342041](https://rip.trb.org/View/2342041).
- Volvo Slippery Road Alert / Hazard Light Alert: cars report low-friction events to Volvo Cloud,
  which decides which segments are slippery and warns approaching cars; a C-ITS service since 2016;
  a DATEX II / SRTI profile item — [DATEX II webinar, Volvo Cars](https://datex2.eu/wp-content/uploads/2023/12/DATEX_II_concl_webinar_10_Volvo_Cars.pdf).
- Volvo-ÅF platform to Trafikverket: friction readings reach Volvo's cloud anonymously in near
  real time; delivered to the authority as monthly compilations in half-hour periods with
  confidence estimates — [Aalborg Univ. Trafikdage paper](https://journals.aau.dk/index.php/td/article/download/5067/4440/16453).
- SRTI / Data for Road Safety: "slippery road" maps to DENM cause code 6 (adverse weather - adhesion)
  with sub-causes — [DATEX II SRTI alignment webinar 2021](https://datex2.eu/wp-content/uploads/2023/12/2021.04.21-webinar-SRTI-alignment-compressed.pdf).
- TomTom Hazard Warnings (white paper, Dec 2023): round-trip latency < 5 s via MQTT push; hazard
  types include SLIPPERY_ROAD ("mainly caused by bad weather but also abnormal liquids (e.g. oil)");
  each message has metadata with creation and expiration time plus severity and location; location
  by WGS84 geo-coordinates or OpenLR (point, linear, area); distribution by Morton-code tiles — [TomTom Hazard Warnings white paper](https://download.tomtom.com/open/banners/White-paper-TomTom-Hazard-Warnings.pdf).
- HERE Hazard Warnings: slippery road, broken-down vehicle, reduced visibility, accidents; ~100-byte
  TPEG messages; 43 countries in NA/EU (2021 one-pager); "seconds rather than minutes"; Road Alerts
  cite data from ~5 million vehicles (2022); HERE Road Alerts meets Data for Road Safety requirements
  for Euro NCAP scoring (July 2026) — [HERE Hazard Warning one-pager 2022](https://assets.here.com/x7rx8ayph7ee/a8adcfe2-d292-4ffc-85fb-8e98c8b6630e/b1f372ce94d412266568c93e06231783/HERE_Hazard_Warning_one_pager_2022.pdf); [HERE Road Alerts Euro NCAP 2026](https://assets.here.com/x7rx8ayph7ee/5g7i6gKZAaZcFq9eCNQ5Qr/1b20c59edcfd854dc212d7b5e1e837c0/20260727_HERE_Road_Alerts_Euro_NCAP.pdf); [HERE blog](https://www.here.com/learn/blog/how-here-hazard-warnings-use-data-to-power-safer-driving). Vendor claims.
- RoadCloud: fuses fleet sensor measurements with in-car data, ML for real-time friction; optical
  sensors distinguish black ice, slush, snow, pooled water; offered on HERE Marketplace (2020) — [Drive Sweden](https://www.drivesweden.net/roadcloud); [HERE press release](https://www.here.com/company/press-releases/en/roadcloud-offer-premium-road-data-here-marketplace); [CORDIS 876812](https://cordis.europa.eu/project/id/876812/reporting).
- Klimator Road Status Information: connected-vehicle data + RWIS + forecasts; trialled by a
  Norwegian road authority; accuracy claim qualitative only; Swedish test claimed 20-30 % winter
  maintenance cost cut — [Traffic Technology Today](https://www.traffictechnologytoday.com/?p=10910); [Almkvist, SIRWEC](https://sirwec.org/wp-content/uploads/2022/04/2.3_Almkvist.pdf); [CORDIS 774187](https://cordis.europa.eu/project/id/774187).
- Teconer RCM411 (optical, mounted on a vehicle): outputs measurements once per second with a
  response time of a few seconds; SD of difference vs the µTEC accelerometer reference 0.10; the
  braking reference meter's absolute accuracy ~0.05 — [Teconer, SIRWEC Helsinki 2012](https://sirwec.org/wp-content/uploads/2022/04/Helsinki-P-14.pdf); [PIARC Andorra 2014](https://proceedings-andorra2014.piarc.org/content/actes/13_eng.pdf). Teconer's own papers; not independent.

### Inferences
- What a planner actually receives is mostly an event ("slippery here, from t0, until t_exp,
  severity") with a linear OpenLR/coordinate extent, not a continuous mu(s) profile. Mapping:
  event present -> M3b "low friction" or "wet" row for the overlapped segments; absence is not
  evidence of dry (coverage depends on traffic: a closed course or remote mountain pass may have no
  fleet traffic at all).
- Accuracy of ~0.1 mu (best published, dedicated sensor) is about the width of M3b's classes
  (wet 0.6-0.8 vs low <= 0.45), so off-board friction can choose a class but not a number; keep the
  class boundaries with hysteresis.
- Passenger-car-derived friction is "friction utilised/available at the slip the car saw", mostly
  low-excitation; it says little about fall-off (post-peak shape) — that must come from the surface
  class table, not from the feed.

### Gaps
- Independent accuracy, latency and false-alarm statistics for NIRA, Volvo, HERE, TomTom, Continental:
  none found beyond vendor claims. Continental's slippery-road features were not researched.
- Segment length / spatial resolution of NIRA friction map layers: only the secondary "<10 m map matching".

## 3. C-ITS / Car2X hazard messages (ETSI DENM)

### Takeaway
DENM carries slippery-road hazards as cause 6 (adverse weather - adhesion) with sub-causes (heavy
frost, ice, black ice, oil, salted road, loose chippings, ...), a validity (default 600 s) and a
coarse relevance distance (<50 m ... >10 km). It is a class-level, short-lived hazard flag, not a
friction value.

### Cited Findings
- Adhesion sub-cause codes include heavy frost (1), ice (5), black ice (6), oil (7), salted road (10);
  later revisions add loose chippings and instant black ice; 0 = unavailable — [ETSI ITS CDD (TS 102 894-2) ASN.1](https://forge.etsi.org/rep/ITS/asn1/cdd_ts102894_2/-/raw/bf0f4cbc44d8c8b90df8fabd79d20333f51f34c1/docs/ETSI-ITS-CDD.md).
- validityDuration DEFAULT defaultValidity = 600 (seconds from detectionTime) — [ETSI LibIts DENM.asn](https://forge.etsi.org/rep/LIBS/LibIts/-/raw/4a5168b1c95fafc44d81b45648a035bb4a90d3b3/asn1/DENM/DENM.asn); [ROS etsi_its_denm_msgs ManagementContainer](https://docs.ros.org/en/jazzy/p/etsi_its_denm_ts_msgs/msg/ManagementContainer.html).
- relevanceDistance enumeration lessThan50m(0) ... lessThan100m, 200m, 500m, 1000m, 5km, over10km(7);
  relevanceTrafficDirection: unavailable, upstream, downstream, all; later versions make relevance
  fields OPTIONAL — [ETSI LibIts DENM.asn](https://forge.etsi.org/rep/LIBS/LibIts/-/raw/4a5168b1c95fafc44d81b45648a035bb4a90d3b3/asn1/DENM/DENM.asn); [ETSI ITS_ASN1 commit](https://forge.etsi.org/rep/ITS/ITS_ASN1/-/commit/b4b4a10c8804a259acfb4029e766d8e0279e47ce).
- EU C-ITS delegated act annex lists the "temporarily slippery road" DENM use case (EN 302 637-3) — [EU C(2019) 1789 annex](https://www.parlament.gv.at/dokument/XXVI/EU/57794/imfname_10887020.pdf).
- ROS 2 message packages exist for DENM (etsi_its_denm_msgs) — [docs.ros.org](https://docs.ros.org/en/iron/p/etsi_its_denm_msgs/index.html).

### Inferences
- A DENM slippery-road event maps directly to M3b's low-friction row over its event zone; the
  sub-cause can choose wet vs ice class. The 600 s default validity is a ready-made staleness bound.

### Gaps
- Field latency of V2V/V2I DENM delivery and deployed density of slippery-road DENM senders: not found.

## 4. Weather nowcasting for road surface state; rain radar; racing practice

### Takeaway
Road surface temperature models (METRo and statistical MOS) forecast pavement temperature at
stations with RMSE ~1.3-1.5 degC (DWD, ~1,100 stations), good enough to flag near-freezing but not
to resolve per-corner state. In F1 the provider (Ubimet; from 2026 Tomorrow.io for the FIA) aims at
minute-level rain timing with forecasts updated every few minutes, but no published minute-ahead
accuracy exists, and misses happen.

### Cited Findings
- DWD evaluation (Nov 2023 - Jan 2024, ~1,100 road weather stations): METRo pavement temperature
  RMSE ~1.5 degC averaged over next-day and next-night periods; MOS 1.3 degC — [EMS2024-148](https://meetingorganizer.copernicus.org/EMS2024/EMS2024-148.html).
- METRo shows diurnal biases in forecast pavement temperature, addressed with an operational hourly
  bias correction — [AMS 94th Annual, paper 235642](https://ams.confex.com/ams/94Annual/webprogram/Paper235642.html).
- Hungarian comparison of RST systems: RMSE ~1.3 to 6 degC across systems; error depends on time of
  day and weather situation — [SIRWEC 6.6 Simon](https://sirwec.org/wp-content/uploads/2022/04/6.6_Simon.pdf).
- Vienna statistical nowcast: daily RMSE ~1 degC, larger at midday — [arXiv 2003.09957](https://pdf.arxiv.org/pdf/2003.09957).
- F1: Ubimet provides the weather service with a portable radar, modelling, satellite and trackside
  stations — [Formula1.com 2016](https://www.formula1.com/en/latest/features/2016/9/how-weather-forecasting-works-in-f1.html); aim to forecast rain timing "to the minute"; data/forecasts updated every few minutes during races — [Ubimet 2015](https://www.ubimet.com/en/formula-1-season-start-variable-weather-conditions-will-again-play-an-important-role/); [Ubimet, ups and downs](https://www.ubimet.com/en/?p=6420).
- 2026: FIA moves to Tomorrow.io satellite data + AI processing (FIA's own accuracy claim) — [The Race](https://www.the-race.com/formula-1/f1-weather-revolution-forecasting-ai/).
- Known miss: weather radar did not predict the severity of rain in Belgium (Domenicali) — [PlanetF1](https://www.planetf1.com/news/stefano-domenicali-weather-radar-belgium).
- Teams also used hilltop mobile radars and occasionally a helicopter ahead of rain fronts — [auto123](https://www.auto123.com/en/news/f1-technique-getting-accurate-weather-forecast-is-essential-to-teams/36271).

### Inferences
- Racing practice treats weather as a lap-to-minutes horizon input that changes strategy (tyres,
  pit timing), not a per-corner friction value; per-corner wetness is learned from the car and the
  track (and marshal/flag information), which supports keeping M3b's surface class per segment from
  the map plus an onboard update, with weather only shifting the whole-window prior.
- METRo itself is open-source from Environment Canada (general knowledge, unverified here).

### Gaps
- Quantitative minute-ahead rain-radar nowcast skill at track scale: not found.
- Road-surface-state (not temperature) nowcast accuracy (wet/dry/ice classification skill): not found.

## 5. Map formats carrying surface / friction

### Takeaway
ASAM OpenDRIVE is the only open map format with a numeric friction field per lane (`<lane><material
surface friction roughness sOffset>`), optionally overridden by an OpenCRG grid with purpose
"friction". Lanelet2 has only a categorical `road_surface` tag; OSM has `surface=*` and
`smoothness=*` (categorical, verifiability contested). None carries fall-off; a planner should map
categories to its own class table (as swb's surfaces.yaml does).

### Cited Findings
- OpenDRIVE: lane surface type, friction and roughness stored in `<material>` inside `<lane>`;
  roughness intended for e.g. sound and motion systems; defaults apply where no material is defined;
  entries ordered by s; no material on the centre lane (spec 11.7.2) — as reported from the [ASAM OpenDRIVE spec, 11.7 lane properties](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/latest/specification/11_lanes/11_07_lane_properties.html) (page returned 404 on direct fetch; content from search summary).
- Rust parser of OpenDRIVE 1.7: material attributes friction, optional roughness, sOffset, optional surface (code) — [opendrive crate, lane/material.rs](https://docs.rs/crate/opendrive/latest/source/src/lane/material.rs).
- OpenDRIVE references OpenCRG via `<CRG>` inside `<surface>`; CRG can carry detailed friction;
  a CRG with purpose friction replaces friction from `<material>` (10.6.5) — [ASAM OpenDRIVE 10.6 Road CRG surface](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/latest/specification/10_roads/10_06_road_surface.html) (404 on direct fetch; from search summary).
- ISPRS 2026: proposes encoding measured road defects as lane-level material changes or as objects;
  CRG emphasises geometry, lane material covers friction and roughness — [ISPRS Archives XLIX-B4-2026-433](https://isprs-archives.copernicus.org/articles/XLIX-B4-2026/433/2026/isprs-archives-XLIX-B4-2026-433-2026.pdf).
- Lanelet2: the only surface attribute is `road_surface` ("dirt, asphalt, concrete, ..."); no
  friction or material attribute; others: subtype, location, speed_limit, participant, one_way,
  region, road_name — [Lanelet2 LaneletAndAreaTagging.md](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md).
- OSM `smoothness=*`: excellent / good / intermediate / bad / very_bad / horrible / very_horrible /
  impassable, defined by which vehicles can use the way (e.g. good: racing bikes, sport motorcycles;
  intermediate: sports cars; bad: normal cars; very_bad: ~18 cm clearance); not rendered on the
  standard map; verifiability was contested (c. 2008) — [OSM wiki Key:smoothness](https://wiki.openstreetmap.org/wiki/Key:smoothness).
- OSM `surface=*` (asphalt, concrete, paving_stones, gravel, dirt, ...) — [OSM wiki Key:surface](https://wiki.openstreetmap.org/wiki/Key:surface) (not fetched in this session; value list from general knowledge, unverified).
- HD map providers: HERE Hazard Warnings / Road Alerts deliver dynamic layers (TPEG) over a map;
  no static friction layer documented in fetched sources — see section 2.

### Inferences
- Consumption pattern for the planner: static class from map (`surface` / `road_surface` /
  OpenDRIVE `material@surface` / swb `surfaces.yaml` label) -> class table (peak mu, fall-off,
  e.g. asphalt dry 1.0+/low fall-off; gravel ~0.6/high fall-off) -> per-node values in the window;
  a numeric `material@friction` or CRG friction, when present, overrides only peak mu, never the
  fall-off; dynamic layers (DENM, vendor hazard) then downgrade the class over their extent until
  expiry. This matches 061's "surface ahead is the map's".
- OSM `smoothness` is a usable proxy for "fall-off / bumpiness" on unpaved roads only; on paved
  roads it does not discriminate grip.

### Gaps
- Exact OpenDRIVE units/ranges for `friction` and `roughness` (spec pages 404 for direct fetch).
- Commercial HD-map static surface/friction layers (HERE HD Live Map, TomTom HD Map): not found.

## 6. Geometry ahead from maps: curvature, grade, width

### Takeaway
OSM centrelines sit typically 2-6 m from survey references (worse in poorly mapped regions), which
is enough to find corners and their turn angle but biases small radii; open DEMs (SRTM, ±16 m
vertical, 30-90 m posts) give poor grade on short segments. Width is rarely tagged in OSM. HD maps
(Lanelet2/OpenDRIVE built from surveys) are the source for R <= 25 m hairpins and width.

### Cited Findings
- Haklay (2010): ~80 % overlap with Ordnance Survey where OSM existed; mean offset ~6 m — [secondary summary, ITC thesis topic](https://kartoweb.itc.nl/msc-carto-thesis/topics/?id=70); paper: Environment and Planning B 37(4) 682-703 (figures from secondary sources; check original).
- Calgary (2023): mean positional offset 2.3 m; >86 % of road segments within 5 m — [ICA Abstracts 6, 124, 2023](https://ica-abs.copernicus.org/articles/6/124/2023/ica-abs-6-124-2023.pdf).
- Africa: 95 % of OSM test data within 8.5 m (Maiduguri), 9.5 m (Nairobi), 13 m (Freetown) of MGCP reference — [ISPRS Archives XLI-B2-615-2016](https://isprs-archives.copernicus.org/articles/XLI-B2/615/2016/isprs-archives-XLI-B2-615-2016.pdf).
- Open DEMs (SRTM, ASTER) lack horizontal resolution / vertical accuracy for road incline;
  OSM-GPS-trace inclines slightly outperform SRTM-1 — [Heidelberg GIScience blog](https://giscienceblog.uni-heidelberg.de/?p=5449).
- SRTM v3 vertical error ±16 m globally (cited) — [FIG 2022 TS04C](https://fig.net/resources/proceedings/fig_proceedings/fig2022/papers/ts04c/TS04C_musa_naiya_et_al_11646.pdf).
- For short road segments accurate gradient from free data is an issue — [U-Shift Declives benchmark](https://github.imc.re/U-Shift/Declives-RedeViaria/blob/main/Benchmark.Rmd).
- GPS + OSM + DEM fusion via nonlinear filter for 3-D road parameters (Boucher & Noyer) — [summary](https://www.bohrium.com/paper-details/a-general-framework-for-3-d-parameters-estimation-of-roads-using-gps-osm-and-dem-data/813070958964244480-4100); crowd-sourced smartphone road grade — [arXiv 2006.03633](https://arxiv.org/pdf/2006.03633).

### Inferences
- A 2-6 m lateral offset on a hairpin of R 9-25 m is 10-60 % of the radius: OSM can say "a >= 150 deg
  corner is coming" reliably but cannot be trusted for the R <= 9 m vs R <= 25 m split in M3b;
  either use a surveyed map / national LiDAR DEM (1-2 m posts) or widen the radius thresholds by the
  map's error. OSM node spacing on curves also quantises curvature (fit a smoothing spline before
  computing kappa).
- Grade > 5 % (M3b downhill row) over 200 m is a 10 m drop: detectable from a 1-5 m LiDAR DEM, marginal
  from SRTM (±16 m). Prefer national LiDAR DEMs (e.g. GSI 5 m in Japan, unverified) or the course's
  own mesh (swb courses).
- Width: OSM `width=*` / `lanes=*` are sparse; treat width as unknown -> conservative (narrow) unless
  from an HD map; 061 already gates the width rule on its width arm.

### Gaps
- Quantified OSM curvature/radius error on mountain roads: no source.
- OSM `width` tag completeness statistics: not found.

## 7. Uncertainty handling: confidence, staleness, fusion, conservative defaults

### Takeaway
Off-board sources expose expiry (DENM validity, TomTom expiration time) and sometimes confidence
(Volvo deliveries to Trafikverket carry confidence estimates); none publishes calibrated error bars.
The robust pattern: map class as prior, off-board hazards as downgrades with expiry, onboard
estimates update the segment just driven, and the most conservative plausible class wins when sources
disagree.

### Cited Findings
- DENM: default validity 600 s from detectionTime — [ETSI LibIts DENM.asn](https://forge.etsi.org/rep/LIBS/LibIts/-/raw/4a5168b1c95fafc44d81b45648a035bb4a90d3b3/asn1/DENM/DENM.asn).
- TomTom: every hazard message has creation and expiration time — [TomTom white paper](https://download.tomtom.com/open/banners/White-paper-TomTom-Hazard-Warnings.pdf).
- Volvo-ÅF to Trafikverket: half-hour periods with confidence estimates — [Aalborg paper](https://journals.aau.dk/index.php/td/article/download/5067/4440/16453).
- Cooperative estimation accuracy depends on number of recent passes (>= 50 vehicles for ~10 % worst-case) — [arXiv:2302.03560](https://arxiv.org/abs/2302.03560).
- NIRA's regional alerts use "multiple car measurements" for high confidence and "negative grip trend monitoring" — [NIRA SIRWEC 2022](https://sirwec.org/wp-content/uploads/2023/10/sirwec2022_5.2_presentation_Zachrisson_Hagg.pdf).

### Inferences (design suggestions for M3b's scheduler, not sourced)
- Represent each node's surface as a class with a probability, not a mu point value; seed set is
  the union over classes with probability above a threshold (cheap: seeds are additive, G always runs).
- Age reports: weight w = exp(-age/tau), tau ~ minutes for wet/ice hazards (DENM default 600 s),
  hours for dry-after-rain; drop at expiry.
- Asymmetric fusion: a slippery report downgrades immediately; upgrading back to dry needs onboard
  evidence or expiry.
- Defaults when nothing is known: map class, worst-case within the weather prior (e.g. surface
  temperature <= 3 degC -> include low-friction seeds); unknown width -> narrowest arm.

### Gaps
- No published standard for confidence fields on friction across vendors; no empirical tau for
  how fast reported wetness/ice decays to irrelevance.
