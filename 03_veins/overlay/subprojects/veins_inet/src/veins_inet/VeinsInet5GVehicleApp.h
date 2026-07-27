#pragma once

#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <vector>
#include <utility>
#include <string>
#include <cmath>

#include "veins_inet/VeinsInetApplicationBase.h"
#include "veins_inet/veins_inet.h"

class VeinsInet5GMessage;

class VEINS_INET_API VeinsInet5GVehicleApp : public veins::VeinsInetApplicationBase {
protected:
    bool startApplication() override;
    bool stopApplication() override;
    void finish() override;
    void processPacket(std::shared_ptr<inet::Packet> pk) override;

private:
    static constexpr int CAV_SPEED_MODE = 0b10100;

    std::string selfVehicleId_;
    std::string sumoVehicleType_;
    std::string manualVehicleType_;
    std::string cavVehicleType_;
    std::string authModeLabel_;
    std::string senderRole_;
    std::string v2vPropagationModel_;
    std::string spatPropagationModel_;
    std::string channelTraceCsv_;
    std::string generalizedPhyCoeffDir_;
    std::string configuredAttackerVehicleId_;
    std::string targetVehicleId_;
    std::string attackDeliveryMode_;
    // SPAT-loss policy: "sumo_fallback" (CACC->ACC) | "safe_stop" (MRM) | "freeze_speed"
    std::string spatLossPolicy_;
    std::string spatCaccTypeId_;   // normal cooperative vType (CACC)
    std::string spatAccTypeId_;    // degraded onboard-sensor vType (ACC)

    bool running_ = false;
    bool bypassAnalyticalChannel_ = false;
    bool activateChannelTrace_ = false;
    bool activateGeneralizedPhyModel_ = false;
    bool losOnly_ = false;   // LOS-only channel: drop building-NLOS / vehicle-NLOSv across all models
    bool enableObuProcessingQueue_ = false;
    bool manualMode_ = true;
    bool isAttacker_ = false;
    bool enableFlooding_ = false;
    bool disguiseFloodAsCam_ = true;
    bool enableSpatControl_ = true;
    bool enableResearchFailsafe_ = true;  // deprecated legacy flag; no longer overrides empty spatLossPolicy_
    bool logGroundtruth_ = false;
    bool logCavLinkTrace_ = false;
    bool logLatencyBreakdown_ = false;
    bool openEndedAttack_ = true;
    bool attackActive_ = false;
    bool attackEndedLogged_ = false;
    bool spatControlActive_ = false;
    bool vehicleBound_ = false;
    bool telemetryStarted_ = false;

    double beaconInterval_ = 0.1;
    double camTrustWindow_ = 300.0;
    double spatTrustWindow_ = 30.0;
    double actualCamInterval_ = 1.0;
    double spatInterval_ = 0.1;
    double actualSpatInterval_ = 1.0;
    double floodRate_ = 100.0;
    double maxActualFloodRate_ = 10.0;
    double attackStartTime_ = 0.0;
    double attackDuration_ = 0.0;
    double targetRange_ = 320.0;
    double pc5AirDelayMean_ = 0.002;
    double pc5AirDelayStd_ = 0.0005;
    double pc5DropRate_ = 0.00001;
    double pc5Range_ = 320.0;
    double pc5HalfDuplexDrop_ = 0.02;
    double sbspsReselectionInterval_ = 0.5;
    double pc5SubchannelsPerSubframe_ = 50.0;
    double pc5ResourcesPerPacket_ = 1.0;
    double pc5CandidateResourceFraction_ = 0.2;
    double pc5TxPowerDbm_ = 23.0;
    double pc5SensingThresholdDbm_ = -94.0;
    double pc5ShadowingStdDb_ = 3.0;
    double pc5NoiseFigureDb_ = 7.0;
    double pc5BandwidthHz_ = 18.72e6;
    double pc5ShadowCorrelationWindowS_ = 1.0;
    double pc5PathLossExponent_ = 2.7;
    double pc5ReferenceLossDb_ = 47.86;
    double spatCarrierFrequencyGHz_ = 5.9;
    double spatBsHeight_ = 10.0;
    double spatUtHeight_ = 1.5;
    double obuCapacityPps_ = 300.0;
    double capacityWindow_ = 1.0;
    double capacityDropSlope_ = 1.0;
    double authDelayMean_ = 0.0;
    double authDelayStd_ = 0.0;
    double authCacheWindow_ = 300.0;
    double authFreshnessWindow_ = 0.5;
    double obuQueueMaxDepth_ = 0.5;
    double processorFreeAt_ = 0.0;
    double spatStaleTimeout_ = 0.5;
    double spatStopDecel_ = 3.0;   // MRM comfortable decel (m/s^2), <= UNECE R157 4 m/s^2
    // Distance to the stop line at which the MRM engages. Explicit rather than emergent: the
    // decel-limited cap sqrt(2*a*d) only constrains a vehicle within v^2/(2a) of the line
    // (~32 m at 13.9 m/s free-flow, 3 m/s^2), so beyond that the manoeuvre was "commanded"
    // while doing nothing at all. Labelling that as MRM counted vehicles halted by ordinary
    // congestion 100-140 m back: 1201 of 1201 MRM-labelled samples sat where the cap could not
    // bind. 35 m is the smallest round distance that still guarantees a comfortable stop from
    // free-flow speed (32.2 m braking + 2 m stop-line margin = 34.2 m).
    double mrmActivationDistance_ = 35.0;
    double spatHandoverTime_ = 1.0;  // SAE J3016 DDT-fallback handover before MRM commits (s); ini-configurable
    // ── Car-following-mode arbiter ───────────────────────────────────────────────
    // The vType (CACC vs ACC) has exactly ONE owner: applyCarFollowingMode(). Each
    // degraded-mode source records its own reason flag and the arbiter resolves them,
    // so the two watchdogs can never fight over setType():
    //   accReasonCam_  — leader CAM lost -> car-following / forward-warning app down
    //   accReasonSpat_ — SPaT lost       -> intersection-assist DDT fallback
    // ACC is held while EITHER reason stands; CACC returns only once both clear.
    bool accReasonCam_ = false;
    bool accReasonSpat_ = false;
    // V2I loss marker for the application-differentiated model. Distinct from accReasonSpat_
    // (which still drives the vType on the LEGACY spatLossPolicy path used by the CV2X /
    // Flooding / WeatherFlooding projects). In the enableCamCacc model the responses map
    // one-to-one onto the applications: ACC <- CAM loss only, MRM <- SPaT loss only. SPaT loss
    // therefore no longer switches the car-following model; it only arms the MRM.
    bool spatLossActive_ = false;
    bool accModeApplied_ = false;    // vType currently set to spatAccTypeId_
    // MRM is REPORTED only once the vehicle has actually come to rest, i.e. once the minimal
    // risk CONDITION is reached. While it is still rolling down the decel-limited approach it
    // is driving on onboard sensors, so its reported state is ACC. Without this the flag was
    // set the moment the manoeuvre was commanded, including far upstream where the cap
    // v = sqrt(2*a*d) is far above free-flow speed and therefore constrains nothing: at
    // 200-400 m from the signal 50-70% of baseline samples carried the flag while driving at
    // 11.5 m/s and never stopping, inflating the reported MRM fraction with inert flag time.
    bool mrmEngaged_ = false;        // MRM is commanding the approach (cap may or may not bind)
    bool mrmStopped_ = false;        // MRM has reached standstill -> this is what is logged
    // Which lost link drove the current degradation (V2V = leader CAM, V2I = SPaT). An MRM is
    // reachable only via V2I loss: no onboard sensor substitutes for a signal phase, whereas
    // losing the leader's CAM only costs the cooperative layer.
    std::string failsafeState() const;    // "CACC" | "ACC" | "MRM"
    std::string failsafeReason() const;   // "" | "V2V" | "V2I" | "V2I+V2V"

    // LINK STATE, recorded independently of the RESPONSE. Needed because activateFailsafe can
    // suppress a response: with ACC disabled the leader-CAM staleness was computed and thrown
    // away, so nothing in the output distinguished "the V2V link was down and we suppressed
    // the reaction" from "the link was fine". These make the comm-layer outcome comparable
    // across all four fail-safe arms, and let analysis condition on link state rather than on
    // whether the vehicle happened to react.
    //   -1 = not applicable (no leader ahead / no SPaT-equipped signal ahead)
    //    0 = fresh
    //    1 = stale
    // -1 matters: "no vehicle ahead" is NOT the same as "the cooperative link is healthy",
    // and the old camFresh=true-when-no-leader conflated the two.
    int camLinkState_ = -1;
    int spatLinkState_ = -1;
    double distToStopLine_ = -1.0;   // logged so MRM engagement is auditable

    // Which fail-safe responses are enabled, from the `activateFailsafe` parameter.
    // "MRM,ACC" = full ladder | "ACC" = degrade only, never stop | "MRM" = stop without the
    // ACC handover stage | "" = legacy, defer to spatLossPolicy.
    std::string activateFailsafe_;
    bool failsafeAcc_ = true;
    bool failsafeMrm_ = true;
    bool failsafeConfigured_ = false;
    double normalMaxSpeed_ = -1.0;
    double lastSpatReceivedAt_ = -1.0;
    // Which junctions are SPaT-equipped at all. This is a STATIC property of the
    // infrastructure, known a priori from the HD map / MAP messages, so it is a run-scoped
    // registry shared by every vehicle, populated the first time ANY vehicle hears SPaT for
    // that signal and never cleared. It must NOT be inferred from one vehicle's own recent
    // receptions: under heavy flooding a CAV can approach an equipped junction having heard
    // nothing from it, and a per-vehicle test then reads that as "unequipped" and suppresses
    // the fallback — so a TOTAL denial of service would produce LESS degradation than a
    // partial one. Function-local static to avoid static-init order issues.
    static std::set<std::string>& spatEquippedTls();
    // When the current next-TLS was acquired. Used as the freshness reference for an equipped
    // junction this vehicle has never personally heard SPaT from (the jamming case), so the
    // clock starts when the junction comes into view rather than never starting at all.
    double tlsAcquiredAt_ = -1.0;
    std::string tlsAcquiredId_;

    // Per-signal SPaT history: TLS id -> last time SPaT for THAT signal was accepted.
    // The intersection-assist app can only FAIL where it was actually being provided, so a
    // junction absent from this map has no RSU coverage (or none yet) and must not trigger
    // the DDT fallback: the CAV simply approaches it as a conventional signal. Without this
    // an unequipped junction produces a spurious MRM on every approach, which in a partly
    // equipped network swamps the attack effect it is supposed to measure.
    std::map<std::string, double> lastSpatFromTls_;
    double frozenSpeed_ = -1.0;
    double prevSpeed_ = -1.0;
    double prevTime_ = -1.0;
    double timeseriesPrevSpeed_ = -1.0;
    double timeseriesPrevTime_ = -1.0;
    bool inBrakingEpisode_ = false;
    double brakeEpisodeDist_ = 0.0;
    double brakeEpisodeMaxDecel_ = 0.0;
    double brakeLastX_ = 0.0;
    double brakeLastY_ = 0.0;
    bool hardBrakeFired_ = false;
    bool emergencyBrakeFired_ = false;
    // TTC-episode conflict state (replaces inCollision_ / stoppedSince_)
    bool inConflictEpisode_ = false;           // currently within a TTC < 3.0 s episode
    bool seriousConflictFiredInEpisode_ = false; // traffic_conflict already fired this episode
    bool tlsViolationLatched_ = false;
    std::string lastRoadId_;
    std::string lastObservedTlsId_;
    char lastObservedTlsSignal_ = 'U';
    double lastObservedTlsDistance_ = -1.0;
    // updateTlsObservation() feeds the fail-safe logic, so it runs unconditionally on the
    // telemetry tick (before the CSV file check). Any violation it detects is latched here
    // and consumed by the row writer, so control no longer depends on logging succeeding.
    int pendingTlsViolation_ = 0;

    // ── Application-differentiated degradation (enableCamCacc) ───────────────────
    // Flooding does not disable "the CAV"; it disables the applications whose feeding
    // link it starves. The two target applications are modelled separately:
    //   leader CAM (V2V) -> car-following / forward-collision-warning app
    //                       loss => CACC drops to onboard-sensor ACC and drives on
    //   SPaT (V2I)       -> intersection-assist app
    //                       loss => DDT fallback (ACC during handover) then MRM stop
    //                       at the stop line, unless the line is already crossed
    bool   enableCamCacc_ = false;
    double camStaleTimeout_ = 0.3;      // leader-CAM staleness window (s)
    double caccLeaderRange_ = 100.0;    // getLeader sensing range (m)
    std::map<std::string, double> lastCamFromVeh_;   // sender SUMO veh id -> last accepted CAM rx time
    void   checkCamCaccFailsafe();      // watchdog: leader-CAM freshness -> CACC/ACC

    std::deque<std::pair<double, double>> rxWindow_;
    double rxWindowLoad_ = 0.0;

    long stat_rx_total_ = 0;
    long stat_rx_delivered_ = 0;
    long stat_drop_base_ = 0;
    long stat_drop_hd_ = 0;
    long stat_drop_sps_ = 0;
    long stat_drop_capacity_ = 0;
    long stat_drop_propagation_ = 0;
    long stat_cam_tx_ = 0;
    long stat_cam_rx_ = 0;
    long stat_spat_rx_ = 0;
    long stat_flood_rx_ = 0;
    long stat_flood_tx_ = 0;
    long stat_target_rx_total_ = 0;
    long stat_target_rx_delivered_ = 0;
    long stat_target_drop_capacity_ = 0;
    long stat_auth_full_ = 0;
    long stat_auth_cachehit_ = 0;
    long stat_auth_stale_drop_ = 0;
    long stat_auth_queue_overflow_ = 0;    // OBU queue overflow drops
    long nextMessageId_ = 1;
    std::map<std::string, double> authCacheExpiry_;

    enum class PolicyDecisionCode {
        Accept,
        RejectQueueOverflow,
        RejectStale,
    };

    struct PolicyDecision {
        PolicyDecisionCode code = PolicyDecisionCode::Accept;
        std::string authReason;
        std::string cacheStatus;
        std::string decisionStage;
        bool updateAuthCache = false;
        double trustWindowOverride = -1.0;

        bool accepts() const { return code == PolicyDecisionCode::Accept; }
    };

    struct ChannelTraceSample {
        double timeS = 0.0;
        double distanceM = std::numeric_limits<double>::quiet_NaN();
        double rxPowerDb = std::numeric_limits<double>::quiet_NaN();
        double dopplerHz = std::numeric_limits<double>::quiet_NaN();
        double delaySpreadS = std::numeric_limits<double>::quiet_NaN();
        double relativeSpeedMps = std::numeric_limits<double>::quiet_NaN();
        double radialVelocityMps = std::numeric_limits<double>::quiet_NaN();
        double pathLossDb = std::numeric_limits<double>::quiet_NaN();
        double losProbability = std::numeric_limits<double>::quiet_NaN();
        double kFactorDb = std::numeric_limits<double>::quiet_NaN();
        double shadowFadingDb = std::numeric_limits<double>::quiet_NaN();
        double smallscaleFadingDb = std::numeric_limits<double>::quiet_NaN();
        double fingerprintGainDb = std::numeric_limits<double>::quiet_NaN();
        double fingerprintDelayNs = std::numeric_limits<double>::quiet_NaN();
        double fingerprintPhaseRad = std::numeric_limits<double>::quiet_NaN();
        bool hasLosFlag = false;
        bool losFlag = false;
        std::string cdlProfile;
    };

    struct ChannelTraceCsvLayout {
        bool legacyOrder = true;
        int txId = -1;
        int rxId = -1;
        int timeS = -1;
        int distanceM = -1;
        int rxPowerDb = -1;
        int dopplerHz = -1;
        int delaySpreadS = -1;
        int relativeSpeedMps = -1;
        int radialVelocityMps = -1;
        int pathLossDb = -1;
        int rssiDbm = -1;
        int losProbability = -1;
        int losFlag = -1;
        int cdlProfile = -1;
        int kFactorDb = -1;
        int shadowFadingDb = -1;
        int smallscaleFadingDb = -1;
        int fingerprintGainDb = -1;
        int fingerprintDelayNs = -1;
        int fingerprintPhaseRad = -1;
    };

    struct ChannelTraceState {
        bool loaded = false;
        bool loadAttempted = false;
        bool logHeaderPrinted = false;
        int loggedLookupCount = 0;
        ChannelTraceCsvLayout layout;
        std::map<std::pair<std::string, std::string>, std::vector<ChannelTraceSample>> samplesByLink;
    };

    struct GeneralizedPhyLinearModel {
        bool loaded = false;
        std::string responseName;
        std::map<std::string, double> coefficients;
    };

    struct GeneralizedPhyBlerCurve {
        bool loaded = false;
        std::string profileName;
        std::vector<double> snrDb;
        std::vector<double> bler;
        std::vector<double> psr;
    };

    // Per-profile small-scale fading distribution parameters.
    // LOS profiles  (CDL-D, CDL-E): Rician — characterised by K (linear) and Ω (mean power).
    // NLOS profiles (CDL-A, CDL-C): Rayleigh — exponential linear power, characterised by Ω.
    //
    // The mean power Ω is distance-conditioned via a log-log power law:
    //   Ω(d) = 10^(omega_log10_intercept + omega_log10_slope * log10(d))
    // Fitted per profile from 5G Toolbox realisations.
    struct ChannelGainDistParam {
        std::string profile;
        std::string distType;            // "rician" or "rayleigh"
        double rician_k_lin        = 0.0; // Linear K factor; 0 for Rayleigh
        double mean_lin            = 1.0; // Global mean linear power Ω (fallback)
        double omega_log10_intercept = 0.0; // a: log10(Ω(d)) = a + b·log10(d)
        double omega_log10_slope     = 0.0; // b
        bool   hasDistanceModel    = false; // true when a,b are available
    };

    struct GeneralizedPhyRuntimeState {
        bool loaded = false;
        bool loadAttempted = false;
        bool hasSplitChannelGainModel = false;
        bool hasBlerCurves = false;
        bool hasGainDistParams = false;
        GeneralizedPhyLinearModel pathLossModel;
        GeneralizedPhyLinearModel rssiModel;
        GeneralizedPhyLinearModel delaySpreadLog10Model;
        GeneralizedPhyLinearModel kFactorModel;
        GeneralizedPhyLinearModel channelGainModel;
        GeneralizedPhyLinearModel channelGainLosModel;
        GeneralizedPhyLinearModel channelGainNlosModel;
        std::map<std::string, GeneralizedPhyBlerCurve>    blerCurvesByProfile;
        std::map<std::string, ChannelGainDistParam>       gainDistByProfile;
    };

    struct GeneralizedPhyPredictors {
        double log10DistanceM = 0.0;
        double relativeSpeedMps = 0.0;
        double radialVelocityMps = 0.0;
        double absRadialVelocityMps = 0.0;
        double headingAlignmentCos = 0.0;
        double losProbability = 0.0;
    };

    // ── NS-3-distilled realism surrogate ("ns3learn"): selected by realismModel.
    // Coefficients distilled from NS-3 5G-LENA teacher runs and loaded from a CSV
    // bundle. Reproduces NS-3's full reliability stack via OMNeT's existing drop
    // hooks: half-duplex (drop_hd), V2V BLER reception (drop_prop), capture-
    // corrected resource collision vs neighbour density (drop_sps). Existing
    // realism methods are untouched; this is purely additive.
    std::string realismModel_;        // "analytical"|"calibrated"|"ns3learn"|"matlablearn"
                                      // |"analytical_m3"|"analytical_col"|"analytical_prop" (Cao+Rehman reference)
    std::string ns3LearnCoeffDir_;
    bool   ns3LearnEnabled_ = false;
    double ns3HalfDuplex_   = 0.0;    // constant half-duplex loss (drop_hd)
    double ns3Capture_      = 0.0;    // fraction of resource collisions that still decode
    double ns3CollL_ = 0.0, ns3CollK_ = 0.0, ns3CollX0_ = 0.0;   // collision logistic (legacy flat model)
    double ns3NoiseFloorDbm_ = -94.3;
    // Distance-aware interference model: SINR_eff(d) = Tx - PL_LOS(d) - noiseFloor - I(density),
    // I(density) = ns3IntfA_*log10(1+CAVneighbours) + ns3IntfB_. Replaces the flat half-duplex+
    // collision losses with an interference-degraded SINR -> reproduces NS-3's distance decay.
    bool   ns3DistanceAware_ = false;
    double ns3IntfA_ = 0.0, ns3IntfB_ = 0.0;
    // Stochastic interference: per-reception penalty I ~ Normal(mu(density), sigma(density)^2),
    // sigma(density) = ns3IntfSigmaA_*log10(1+n) + ns3IntfSigmaB_. The variance smears the BLER
    // threshold so PDR decays GRADUALLY with distance (matching NS-3) rather than as a hard cliff.
    bool   ns3Stochastic_ = false;
    double ns3IntfSigmaA_ = 0.0, ns3IntfSigmaB_ = 0.0;
    // Decomposed cascade (adopted model): each NR mechanism is its own per-packet stage —
    //   (1) half-duplex drop (pc5HalfDuplexDrop_),
    //   (2) SB-SPS collision: with prob collisionP(n)=sigmoid(c0+c1 n+c2 n^2), SINR -= Normal(capMu,capSigma) [capture],
    //   (3) decode/BLER: deliver with prob decodeP(SINR)=sigmoid(d0+d1 SINR+d2 SINR^2).
    bool   ns3Cascade_ = false;
    double ns3DecC0_=0.0, ns3DecC1_=0.0, ns3DecC2_=0.0;   // decode logistic (vs SINR dB), combined (legacy)
    // GROUNDED capture: separate measured decode curves for non-colliding vs colliding receptions
    // (decode_coll rises with SINR = capture). Replaces the fitted constant penalty.
    // density-aware: sigmoid(c0 + c1*SINR + c2*SINR^2 + c3*n + c4*SINR*n) — capture weakens with density
    double ns3DecNcC0_=0.0, ns3DecNcC1_=0.0, ns3DecNcC2_=0.0, ns3DecNcC3_=0.0, ns3DecNcC4_=0.0;   // decode | no collision
    double ns3DecCcC0_=0.0, ns3DecCcC1_=0.0, ns3DecCcC2_=0.0, ns3DecCcC3_=0.0, ns3DecCcC4_=0.0;   // decode | collision (capture)
    double ns3ColC0_=0.0, ns3ColC1_=0.0, ns3ColC2_=0.0;   // baseline collision logistic (vs CAV density n)
    // Flood/load-aware contention (held-out-validated, MAE 0.013 vs NS-3): an in-range flooding
    // attacker adds a bounded per-interferer factor q_a(floodRate,n) on top of the baseline:
    //   collision = 1 - (1 - base(n)) * PROD_attackers (1 - q_a(rate,n));  zero attackers => base(n).
    double ns3QaB0_=0.0, ns3QaB1_=0.0, ns3QaB2_=0.0, ns3QaB3_=0.0;   // q_a = sigmoid(b0+b1 log10(rate)+b2 log10(n+1)+b3 log10(rate)log10(n+1))
    double ns3CapMu_=0.0, ns3CapSigma_=0.0;               // capture interferer penalty (dB)
    double ns3HdC0_=0.0, ns3HdC1_=0.0, ns3HdC2_=0.0;      // half-duplex logistic (vs CAV density n), learned from NS-3 MAC
    double cascadeHalfDuplexDrop() const;                 // hd(n) = sigmoid(hd_c0 + hd_c1 n + hd_c2 n^2)
    GeneralizedPhyBlerCurve ns3BlerPssch_;   // PSSCH data decode vs SINR
    GeneralizedPhyBlerCurve ns3BlerSci_;     // SCI-2 control decode vs SINR
    bool loadNs3LearnModel(const std::string& dir);
    double ns3LearnReception(double distance3dMeters, const std::string& senderId,
                             double relX, double relY) const;
    // Per-link (per-sender, receiver-side) V2V channel cache for TR 37.885 spatial coherence:
    // holds the LOS/NLOSv state + AR(1)-correlated shadow fading, decorrelating over the
    // correlation distance (LOS 10 m / NLOSv 13 m). Mutable so it updates from the const RX path.
    struct V2vChanState { bool init=false; bool los=true; double nlosvExtra=0.0; double shadow=0.0;
                          double lastRelX=0.0, lastRelY=0.0; };
    mutable std::map<std::string, V2vChanState> v2vChan_;
    double v2vChannelPathLossDb(const std::string& senderId, double distance3dMeters,
                                double fcGhz, double relX, double relY) const;
    double ns3LearnCollisionDrop() const;
    int    countNeighboursInRange() const;
    double inRangeAttackerFloodSurvival(double n) const;   // PROD_attackers (1 - q_a(floodRate,n)); 1.0 if none
    double ns3V2vPathLossDb(double distance3dMeters, double fcGhz) const;   // per-link sampled LOS/NLOSv

    // ── Combined Analytical Reference (paper Eq. m3): a faithful, teacher-free
    // baseline assembled WITHOUT double counting from two published NR Mode-2
    // models — Cao 2026 (SB-SPS collision + half-duplex, density) and Rehman 2023
    // (noise-limited decode, distance):
    //   PDR = (1 - P_HD) * (1 - P_COL(n)) * g(SNR(d)).
    // Selected via realismModel = "analytical_m3" (both factors), "analytical_col"
    // (Cao contention-only ablation) or "analytical_prop" (Rehman propagation-only
    // ablation). Purely additive; the "analytical"/"ns3learn" paths are untouched.
    // No double counting: Model 2 uses SNR (noise only, empty interferer set), so
    // all multi-vehicle contention lives ONLY in Model 1's P_COL, and half-duplex
    // ONLY in Model 1's P_HD.
    bool   analyticalM3Enabled_ = false;   // any of the three m3 modes active
    bool   m3UseCollision_ = false;        // Cao P_COL + P_HD active (m3, col)
    bool   m3UseDecode_    = false;        // Rehman g(SNR(d)) active   (m3, prop)
    double m3Nr_   = 124.0;                // candidate SL resources = window slots x (9/12) x N_sc (Cao Eq.1)
    double m3Pi0_  = 1.0 / 11.0;           // Cao stationary reselection prob (Rc ~ U[5,15])
    double m3Nc_   = 2.0;                   // packets per collided resource (under-saturation)
    double m3PHd_  = 0.01;                  // half-duplex = t_s / T_RRI (Cao)
    double m3SigmaLosDb_  = 3.0;            // TR 37.885 V2V-Urban shadowing std (LOS)
    double m3SigmaNlosDb_ = 3.0;            // TR 37.885 V2V-Urban shadowing std (NLOSv base)
    double m3NlosvExcessStdDb_ = 4.5;       // TR 37.885 NLOSv vehicle-blockage excess std
    GeneralizedPhyBlerCurve m3BlerCurve_;   // independent 5G-Toolbox CDL PER(x) for Model 2
    mutable std::map<int, double> m3DecodeCache_;   // g(SNR(d)) memoized by round(d)
    mutable std::map<int, double> m3CollCache_;     // P_COL(n)  memoized by n
    bool   loadAnalyticalM3Model();                          // load CDL curve + derive N_r, P_HD
    double caoCollisionProb(int n) const;                   // Model 1: fixed-point P_COL(n), p_k=0
    double rehmanDecodeProb(double distanceM) const;        // Model 2: noise-limited g(SNR(d))

    void readParameters();
    static ChannelTraceState& channelTraceState();
    static GeneralizedPhyRuntimeState& generalizedPhyRuntimeState();
    static void ensureChannelTraceLoaded(const std::string& csvPath);
    static void ensureGeneralizedPhyModelLoaded(const std::string& coeffDir);
    static bool loadGeneralizedPhyLinearModel(const std::string& csvPath, GeneralizedPhyLinearModel& model);
    static bool loadGeneralizedPhyBlerCurve(const std::string& csvPath,
                                            const std::string& profileName,
                                            GeneralizedPhyBlerCurve& curve);
    static bool parseChannelTraceRow(const std::string& line,
                                     const ChannelTraceCsvLayout& layout,
                                     std::string& txId,
                                     std::string& rxId,
                                     ChannelTraceSample& sample);
    static std::vector<std::string> splitCsvLine(const std::string& line);
    const ChannelTraceSample* lookupChannelTraceSample(const std::string& txId,
                                                       const std::string& rxId,
                                                       double timeS) const;
    void maybeLogChannelTraceSample(const std::string& txId,
                                    const std::string& rxId,
                                    double timeS,
                                    const char* context) const;
    void logCavLinkTrace(inet::Ptr<const VeinsInet5GMessage> payload) const;
    bool initialiseVehicleState();
    void scheduleVehicleBindRetry();
    void scheduleCavStartup();
    void activateCavMode();
    void startTelemetry();
    void scheduleTelemetry();
    void scheduleCam(bool first = false);
    void scheduleFlood();
    void scheduleSpatWatchdog();
    bool targetAvailableForFlood() const;
    std::string resolveAttackTargetId(const char* kind) const;
    void sendCam();
    void sendFloodCam();
    // senderIdOverride: non-empty overrides selfVehicleId_ in the message.
    // posXOverride / posYOverride: sentinel -1e38 means use current real position.
    void send5GMessage(const char* kind,
                       const std::string& targetId = "",
                       const std::string& senderIdOverride = "",
                       double posXOverride = -1e38,
                       double posYOverride = -1e38);
    void deliverAfterAirDelay(inet::Ptr<const VeinsInet5GMessage> payload);
    void authenticateAndApplyMessage(inet::Ptr<const VeinsInet5GMessage> payload, double timeReceived, double speedAtReceipt);
    void applyDeliveredMessage(inet::Ptr<const VeinsInet5GMessage> payload,
                               double timeReceived,
                               double speedAtReceipt,
                               double configuredDelay,
                               const std::string& authReason,
                               const std::string& cacheStatus = "",
                               const std::string& decisionStage = "");
    void applySpat(char phase);
    void checkSpatStaleness();
    void setSafeMaxSpeed(double speed);
    void restoreSumoBehavior();
    void logVehicleTimeseries();
    void updateTlsObservation(int& tlsViolationEvent);
    int lookupLeaderIsCav(const std::string& leaderId) const;
    double lookupLeaderSpeed(const std::string& leaderId) const;
    void logSpatResponse(inet::Ptr<const VeinsInet5GMessage> payload, double timeReceived, double timeActed, double speedAtReceipt, double configuredDelay, const std::string& authReason) const;
    void logSpatAuthDecision(inet::Ptr<const VeinsInet5GMessage> payload,
                             const PolicyDecision& decision,
                             double timeReceived,
                             double decisionTime,
                             double speedAtReceipt,
                             double configuredDelay) const;
    void logLatencyBreakdown(inet::Ptr<const VeinsInet5GMessage> payload,
                             bool accepted,
                             double timeReceived,
                             double decisionTime,
                             double appliedTime,
                             double speedAtReceipt,
                             double configuredDelay,
                             const std::string& authReason,
                             const std::string& cacheStatus,
                             const std::string& decisionStage) const;
    PolicyDecision runPolicyDecisionPoint(inet::Ptr<const VeinsInet5GMessage> payload,
                                          double decisionTime,
                                          const std::string& authReason,
                                          const std::string& cacheStatus);
    bool runPolicyEnforcementPoint(inet::Ptr<const VeinsInet5GMessage> payload,
                                   const PolicyDecision& decision,
                                   double timeReceived,
                                   double decisionTime,
                                   double speedAtReceipt,
                                   double configuredDelay,
                                   const std::string& cacheKey);
    void recordPolicyDecisionCounters(const PolicyDecision& decision);

    // OBU processing queue model (M/D/1 approximation)
    double scheduleAuthDelay(double serviceTime);   // returns queue-adjusted relative delay; -1 = overflow

    bool isForThisVehicle(inet::Ptr<const VeinsInet5GMessage> payload) const;
    bool isTargetVehicle() const;
    bool passesAnalyticalPdr(inet::Ptr<const VeinsInet5GMessage> payload);
    void writeDistancePdrCsv();
    void writeFloodCollCsv();
    double getPacketRateHz() const;
    double getActualCamInterval() const;
    double getActualFloodInterval() const;
    double getEffectiveSpatStaleTimeout() const;
    double getMrmTriggerTimeout() const;   // last-SPaT age before the MRM commits
    void applyCarFollowingMode();           // sole owner of the CACC/ACC vType switch
    void degradeToAcc();                    // CACC -> ACC graceful degradation (SPaT reason)
    void recoverFromAccFallback();          // ACC -> CACC on fresh SPaT (clears SPaT reason)
    void runIntersectionAssistFallback(double spatAge);   // SPaT lost: ACC handover, then MRM
    void runMinimalRiskManeuver();          // decel-limited stop AT the stop line, dilemma-zone aware
    // Decel-limited approach-to-rest at the next stop line, shared by the SPaT red/yellow
    // response and the MRM. Returns the speed cap for this tick, or a negative sentinel when
    // the cap must be released: -2 = inside the junction box or the line is already behind,
    // -1 = dilemma zone (cannot stop within spatStopDecel, so commit and clear).
    double stopLineApproachSpeed(double* distanceOut = nullptr) const;
    double getMessageLoadUnits(const std::string& kind) const;
    double getPc5NoiseFloorDbm() const;
    double getPacketSensingRatio(double distanceMeters) const;
    double getLinkPacketSensingRatio(inet::Ptr<const VeinsInet5GMessage> payload) const;
    double getTraceAdjustedPacketSensingRatio(const ChannelTraceSample& sample) const;
    double getTraceAdjustedAirDelay(inet::Ptr<const VeinsInet5GMessage> payload) const;
    double sampleRuntimeShadowFadingDb(const std::string& txId,
                                       const std::string& rxId,
                                       double timeS,
                                       bool losFlag) const;
    double sampleFadingGainDb(const std::string& cdlProfile,
                               const std::string& txId,
                               const std::string& rxId,
                               double timeS,
                               double distanceM) const;
    static bool loadChannelGainDistParams(const std::string& csvPath,
                                          std::map<std::string, ChannelGainDistParam>& out);
    static double interpolateBlerCurve(const GeneralizedPhyBlerCurve& curve, double snrDb);
    bool buildGeneralizedPhySample(inet::Ptr<const VeinsInet5GMessage> payload, ChannelTraceSample& sample) const;
    GeneralizedPhyPredictors deriveGeneralizedPhyPredictors(inet::Ptr<const VeinsInet5GMessage> payload) const;
    double evaluateGeneralizedPhyModel(const GeneralizedPhyLinearModel& model, const GeneralizedPhyPredictors& predictors) const;
    double get3gppUrbanLosProbability(double distanceMeters, bool uma) const;
    double get3gppUrbanPathLossDb(double distance2dMeters, double distance3dMeters, bool uma, bool los, double txHeightMeters, double rxHeightMeters) const;
    double getTotalResourcePool() const;
    double estimateChannelBusyRatio() const;
    double estimateSpsCollisionDrop(inet::Ptr<const VeinsInet5GMessage> payload) const;
    double capacityDropProbability(double loadUnits);
    double sampleAuthDelay() const;
    bool isMessageStale(double timeSent, double decisionTime) const;
    bool hasValidAuthCache(const std::string& cacheKey) const;
    void updateAuthCache(const std::string& cacheKey, double timeActed, double trustWindowOverride = -1.0);
    double authTrustWindowFor(inet::Ptr<const VeinsInet5GMessage> payload) const;
    std::string authCacheKeyFor(inet::Ptr<const VeinsInet5GMessage> payload) const;
    inet::Coord currentPosition() const;
    double currentSpeed() const;
    int getSchemaVehicleIndex() const;
    int getSchemaObuId() const;
    bool isSchemaAttacker() const;
    void logSchemaSelfState() const;
    void logSchemaGroundTruth(inet::Ptr<const VeinsInet5GMessage> payload) const;
    void logSchemaReceived(inet::Ptr<const VeinsInet5GMessage> payload) const;
};
