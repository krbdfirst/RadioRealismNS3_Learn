package org.car2x.veins.subprojects.veins_inet;

import org.car2x.veins.subprojects.veins_inet.VeinsInetApplicationBase;

simple VeinsInet5GVehicleApp extends VeinsInetApplicationBase
{
    parameters:
        @class(VeinsInet5GVehicleApp);

        string manualVehicleType = default("veh_human");
        string cavVehicleType = default("veh_av");
        string authModeLabel = default("INET_5G");
        // Per-message-type auth cache lifetime (falls back to authCacheWindow).
        double camTrustWindow @unit(s) = default(300s);
        double spatTrustWindow @unit(s) = default(30s);

        // ── OBU single-threaded processing queue model ─────────────────────
        // Replaces the independent-timer model with a realistic M/D/1 queue.
        // When the queue backlog exceeds obuQueueMaxDepth, new messages are dropped.
        bool enableObuProcessingQueue = default(false);
        double obuQueueMaxDepth @unit(s) = default(0.5s);

        double beaconInterval @unit(s) = default(0.1s);
        double actualCamInterval @unit(s) = default(1s);
        double spatInterval @unit(s) = default(0.1s);
        double actualSpatInterval @unit(s) = default(1s);
        double pc5AirDelayMean @unit(s) = default(0.002s);
        double pc5AirDelayStd @unit(s) = default(0.0005s);
        double pc5DropRate = default(0.00001);
        double pc5Range @unit(m) = default(320m);
        double pc5HalfDuplexDrop = default(0.02);
        double sbspsReselectionInterval @unit(s) = default(0.5s);
        double pc5SubchannelsPerSubframe = default(50);
        double pc5ResourcesPerPacket = default(1);
        double pc5CandidateResourceFraction = default(0.2);
        double pc5TxPowerDbm = default(23);
        double pc5SensingThresholdDbm = default(-94);
        double pc5ShadowingStdDb = default(3);
        double pc5NoiseFigureDb = default(7);
        double pc5BandwidthHz = default(18.72e6);
        double pc5ShadowCorrelationWindow @unit(s) = default(1s);
        double pc5PathLossExponent = default(2.7);
        double pc5ReferenceLossDb = default(47.86);
        // Optional MATLAB-generated NR PC5 trace CSV. When set, the app loads
        // per-link PHY descriptors (rx power / Doppler / delay spread).
        // activateChannelTrace controls whether the trace only loads for
        // diagnostics, or also drives the propagation/PDR step for matched links.
        string channelTraceCsv = default("");
        bool activateChannelTrace = default(false);
        // Optional generalized NR-PC5 PHY model exported as coefficient CSVs.
        // When enabled, OMNeT computes PHY features from live geometry using
        // the fitted equations instead of replaying exact tx-rx-time samples.
        string generalizedPhyCoeffDir = default("");
        bool activateGeneralizedPhyModel = default(false);
        // ── NS-3-distilled realism surrogate selector (additive; existing methods
        // are unchanged). "analytical"|"calibrated"|"matlablearn" keep using the
        // flags/params above; "ns3learn" loads NS-3-distilled coefficients from
        // ns3LearnCoeffDir and reproduces NS-3's BLER + resource-collision +
        // half-duplex through the existing drop pipeline.
        string realismModel = default("analytical");
        string ns3LearnCoeffDir = default("");
        // ── Combined Analytical Reference (Cao 2026 + Rehman 2023), paper Eq. m3 ──
        // realismModel = "analytical_m3" applies (1-P_HD)(1-P_COL(n))*g(SNR(d));
        //              = "analytical_col"  Cao contention-only ablation (density);
        //              = "analytical_prop" Rehman propagation-only ablation (distance).
        // Model 1 = Cao's PUBLISHED closed-form (arXiv 2309.16680); equations + pi0/N_c
        // from the paper, config inputs below set to our scenario (comparable to NS3Learn)
        // and within Cao Table II. Model 2 (Rehman) reads an INDEPENDENT CDL BLER curve.
        // Nothing is fitted to the ground truth.
        string m3BlerCurveCsv = default("");              // independent CDL PER(x) curve (MCS 8)
        double m3SelectionWindowSlots = default(33);      // scenario T2 window (Cao Eq.1 note: 1..T_RRI)
        double m3SlSlotFraction = default(0.75);          // SL slots / total = 9/12 (slBitMap); Cao counts SL slots
        double m3NumSubchannels = default(5);             // our channel N_sc (10 MHz/15 kHz); Cao demo=2
        double m3SlotDurationMs = default(1.0);           // t_s = 1 ms  (Cao Table II)
        double m3RriMs = default(100.0);                  // T_RRI = 100 ms (Cao Table II)
        double m3Nc = default(2.0);                       // N_c = 2 (Cao under-saturation regime)
        // LOS-only channel: drop building-NLOS (analytical/generalized) and vehicle-NLOSv
        // (ns3learn) so every model uses pure line-of-sight propagation. Pairs with the
        // NS-3 UMi_StreetCanyon_LoS reference for a forced-LOS, like-for-like comparison.
        bool losOnly = default(false);
        string v2vPropagationModel = default("tr38901_umi");
        string spatPropagationModel = default("tr38901_umi");
        double spatCarrierFrequencyGHz = default(5.9);
        double spatBsHeight @unit(m) = default(10m);
        double spatUtHeight @unit(m) = default(1.5m);

        // Receiver-side OBU processing limit. Flooded CAMs, normal CAMs, and
        // SPaT all compete for this same processing budget.
        double obuCapacityPps = default(300);
        double capacityWindow @unit(s) = default(1s);
        double capacityDropSlope = default(1);
        double authDelayMean @unit(s) = default(0s);
        double authDelayStd @unit(s) = default(0s);
        double authCacheWindow @unit(s) = default(300s);
        double authFreshnessWindow @unit(s) = default(0.1s);
        double spatStaleTimeout @unit(s) = default(0.5s);
        // MRM comfortable deceleration magnitude (m/s^2) used to compute the kinematic
        // approach-to-rest at the stop line and the dilemma-zone threshold. 3.0 m/s^2 is
        // within the UNECE R157 ALKS MRM envelope (<= 4 m/s^2).
        double spatStopDecel = default(3.0);
        // Distance to the stop line at which the MRM engages (m). Beyond it the decel-limited
        // cap sqrt(2*a*d) exceeds free-flow speed and constrains nothing, so the manoeuvre is
        // neither applied nor labelled. 35 m is the smallest round value that still allows a
        // comfortable stop from 13.9 m/s at 3 m/s^2 (32.2 m braking + 2 m margin).
        double mrmActivationDistance @unit(m) = default(35m);
        // SAE J3016 DDT-fallback handover/transition period (s). For the mrm/"safe_stop"
        // policy the MRM only commits once SPaT has been absent for
        //   actualSpatInterval (loss detection) + spatHandoverTime
        // i.e. one message interval plus this handover window. During the window the CAV
        // degrades to onboard-sensor ACC (drive on, attempt recovery). Override per-run in
        // the ini (e.g. *.node[*].app[0].spatHandoverTime = 10s).
        double spatHandoverTime @unit(s) = default(1s);
        // CACC (normal cooperative) and ACC (degraded onboard-sensor) vType ids used for
        // the CACC->ACC graceful degradation (SAE J3161/1 C-V2X / SR-CACC sensor fallback).
        string spatCaccTypeId = default("veh_av");
        string spatAccTypeId = default("veh_av_acc");
        bool enableSpatControl = default(true);
        // ── Application-differentiated degradation ───────────────────────────────────
        // Flooding does not disable "the CAV"; it disables the applications whose feeding
        // link it starves. The two target applications degrade independently:
        //   leader CAM (V2V) -> car-following / forward-collision-warning app.
        //     Loss costs the cooperative capability only: CACC drops to onboard-sensor ACC
        //     (larger headway) and the CAV keeps driving, still tracking the vehicle ahead.
        //     Location independent, and it never stops the vehicle on its own.
        //   SPaT (V2I)       -> intersection-assist app.
        //     Loss means the CAV cannot read the signal it is approaching, so it runs the
        //     SAE J3016 DDT-fallback ladder CACC -> ACC -> MRM: ACC through the
        //     spatHandoverTime window, then a spatStopDecel stop AT the stop line. Exempt
        //     if the stop line is already crossed or it is inside the junction box, where
        //     it clears the intersection in ACC instead.
        // Both reasons feed one vType arbiter, so they cannot undo each other. When this is
        // true, spatLossPolicy is not consulted for SPaT loss (the ladder above replaces it).
        // Off by default so the comm-side runs are unaffected.
        bool enableCamCacc = default(false);
        double camStaleTimeout @unit(s) = default(0.5s);   // 5 missed CAM at 10 Hz, matching the SPaT window
        double caccLeaderRange @unit(m) = default(100m);   // getLeader sensing range for the follower link
        // Deprecated and unread: the combined "blind at the junction" stop it triggered has
        // been replaced by the SPaT-loss MRM above. Declared only so existing inis still parse.
        double blindStopDistance @unit(m) = default(5m);
        // Degraded-mode SPAT-loss policy (IEEE 1609.2 / SAE J2945/1 degraded operation).
        //   "sumo_fallback" — CACC->ACC graceful degradation (SAE J3161/1 / SR-CACC):
        //                     switch to the onboard-sensor ACC vType, stay until SPaT returns.
        //   "safe_stop"     — SAE J3016 DDT fallback -> minimal-risk maneuver (MRM): degrade
        //                     to ACC during the handover window, then a controlled
        //                     spatStopDecel stop AT the stop line if SPaT is still absent.
        //   "freeze_speed"  — hold last SPAT-commanded speed ceiling until SPAT resumes.
        //   ""              — no explicit SPAT-loss recovery; the last SPAT-commanded
        //                     TraCI speed limit remains in effect until a fresh SPAT arrives.
        // Deprecated legacy alias: enableResearchFailsafe is retained for backward
        // compatibility, but no longer overrides an empty spatLossPolicy.
        // Which fail-safe responses this CAV is allowed to perform. Comma-separated, order
        // and case insensitive:
        //   "MRM,ACC" - both application failure responses active:
        //               V2V loss (stale leader CAM) -> CACC drops to onboard-sensor ACC;
        //               V2I loss (SPaT) at a signal -> ACC handover, then minimal-risk stop.
        //   "ACC"     - the CAR-FOLLOWING / forward-collision-warning application is under
        //               study. V2V loss (stale leader CAM) degrades CACC -> ACC. V2I loss has
        //               NO modelled consequence: that application is not under study, so the
        //               CAV approaches the signal conventionally and never stops.
        //   "MRM"     - the INTERSECTION-ASSIST application is under study. V2I loss runs the full
        //               ladder INCLUDING its ACC handover, because a real ADS degrades to
        //               onboard sensors and attempts recovery before committing to a stop -
        //               the handover is intrinsic to the DDT fallback, not optional. What
        //               this setting drops is the V2V-driven degradation.
        //   ""        - NO FAIL-SAFE WAS DESIGNED. The CAV does not degrade and does not
        //               stop, however much of the link it loses. A deliberate experimental
        //               arm (the un-mitigated CAV), not a fallback to another mechanism.
        // Applies to the application-differentiated model (enableCamCacc). spatLossPolicy
        // below is a SEPARATE legacy mechanism, still used by the CV2X / Flooding /
        // WeatherFlooding projects, and is not consulted when enableCamCacc is true.
        string activateFailsafe = default("");
        string spatLossPolicy = default("");
        bool enableResearchFailsafe = default(true);
        // When true, skip radio-layer analytical drops (half-duplex, path-loss,
        // SPS, base-drop-rate) and keep only the OBU capacity model.
        // Set to true when Ieee80211 radio handles the channel; set to false
        // when IdealWirelessNic is used and the analytical model is the sole PDR model.
        bool bypassAnalyticalChannel = default(false);

        bool enableFlooding = default(false);
        bool disguiseFloodAsCam = default(true);
        string attackerVehicleId = default("");
        string targetVehicleId = default("");
        // Interferer message delivery mode:
        //   "auto"      — FLOOD_CAM targets targetVehicleId
        //   "unicast"   — deliver only to targetVehicleId (if configured)
        //   "multicast" — broadcast to all eligible receivers
        string attackDeliveryMode = default("auto");
        bool logGroundtruth = default(false);
        bool logCavLinkTrace = default(false);
        bool logLatencyBreakdown = default(false);
        double floodRate = default(100);
        double maxActualFloodRate = default(10);
        double attackStartTime @unit(s) = default(0s);
        double attackDuration @unit(s) = default(0s);
        double targetRange @unit(m) = default(320m);
}
