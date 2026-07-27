#pragma once
//
// Car5G.h
//
// CV2X 5G PC5 Sidelink Mode 4 vehicle application.
//
// 3GPP References:
//   TS 36.213 §14   — LTE-V2X resource selection (Mode 4 / SB-SPS)
//   TR 37.885       — NR V2X study item, air-latency and reliability targets
//   TS 38.321 §5.22 — NR sidelink resource selection (Mode 2)
//
// Three Mode 4 behaviours modelled
// ─────────────────────────────────────────────────────────────
// 1. AIR-INTERFACE LATENCY
//    Every received message is held for truncnormal(2ms, 0.5ms)
//    before delivery to the application layer. Models OFDM encoding,
//    propagation, and decoding on the PC5 sidelink channel.
//    Source: TR 37.885 Table 6-1.
//
// 2. SB-SPS / COLLISION PROBABILITY
//    Collision probability is estimated with a local analytical
//    approximation inspired by:
//    Gonzalez-Martin et al., "Analytical Models of the Performance
//    of C-V2X Mode 4 Vehicular Communications", IEEE, 2018.
//    The implementation uses:
//      eq. (21)  pSIM weighting between Step 2 and Step 3
//      eq. (22)  alpha(CBR)
//      eq. (24)  p_s(d)
//      eq. (28)  N resources in the selection window
//      eq. (29)  excluded resources / CBR approximation
//      eq. (34)  CBR = E / N
//    Distance-dependent propagation/interference remains handled by
//    the existing Veins PHY/MAC stack below the application layer.
//
// 3. HALF-DUPLEX COLLISION DROP
//    A Mode 4 UE transmitting on resource block R_i cannot receive
//    on R_i simultaneously. This is modeled following:
//      eq. (7)  delta_HD = lambda / 1000
//    from the same IEEE 2018 analytical paper.
//
// Extension point — onPc5Delivery()
// ─────────────────────────────────────────────────────────────
// The virtual method onPc5Delivery(frame) is called when a frame has
// survived the full PC5 drop pipeline AND the air-delay timer fires.
// Default: forwards to TraCIDemo11p::onWSM(frame).
// Subclasses may override it to inject post-PC5 processing —
// matching real protocol stack layering.
//

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <map>

namespace veins {

class Car5G : public TraCIDemo11p {

public:
    void initialize(int stage) override;
    void finish()              override;

protected:
    void onWSM(BaseFrame1609_4* frame)      override;
    void handleSelfMsg(cMessage* msg)       override;
    void handlePositionUpdate(cObject* obj) override;

    /**
     * Called when a frame has survived the full PC5 drop pipeline
     * (half-duplex + SB-SPS collision + base drop) and the air-delay
     * timer has fired.
     *
     * Default: calls TraCIDemo11p::onWSM(frame)  — baseline unchanged.
     *
     * Overriding here (rather than in onWSM) keeps the layering honest:
     *   (a) MAC-dropped frames are never seen   → correct row counts
     *   (b) No double row writing               → one row per delivery
     *   (c) Protocol stack order respected      → MAC layer below app
     *
     * Ownership: caller deletes frame after return.  Implementations
     * that need to retain the frame must call frame->dup().
     */
    virtual void onPc5Delivery(BaseFrame1609_4* frame);

    // ── Parameter group 1: Air-interface latency ─────────────
    double pc5AirDelayMean_;
    double pc5AirDelayStd_;
    double pc5DropRate_;
    double pc5Range_;

    // ── Parameter group 2: Analytical SB-SPS / collision model ─
    double sbspsReselectionInterval_;
    double sbspsReselectionSilence_;
    double pc5SubchannelsPerSubframe_;
    double pc5ResourcesPerPacket_;
    double pc5CandidateResourceFraction_;
    double pc5TxPowerDbm_;
    double pc5SensingThresholdDbm_;
    double pc5ShadowingStdDb_;
    double pc5PathLossExponent_;
    double pc5ReferenceLossDb_;

    // ── Parameter group 3: Half-duplex collision ─────────────
    double pc5HalfDuplexDrop_;

    // ── Pending message store (air-delay queue) ───────────────
    struct Pc5Msg {
        BaseFrame1609_4* frame;
        double           timeReceived;
    };
    std::map<cMessage*, Pc5Msg> pendingPc5_;

    double getPacketRateHz() const;
    double getPacketSensingRatio(double distanceMeters) const;
    double getTotalResourcePool() const;
    double estimateChannelBusyRatio(const Coord& receiverPos, cModule* selfHost) const;
    double estimateSpsCollisionDrop(BaseFrame1609_4* frame, const Coord& receiverPos, cModule* selfHost) const;

    // ── Statistics ────────────────────────────────────────────
    long stat_rx_total_         = 0;
    long stat_rx_dropped_base_  = 0;
    long stat_rx_dropped_hd_    = 0;
    long stat_rx_dropped_sps_   = 0;
    long stat_rx_delayed_       = 0;
};

} // namespace veins
