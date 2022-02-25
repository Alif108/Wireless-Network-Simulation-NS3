/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 ResiliNets, ITTC, University of Kansas
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Truc Anh N. Nguyen <annguyen@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 */

#include "tcp-vegas-w.h"
#include "tcp-socket-state.h"

#include "ns3/log.h"


// ----------------------- custom ------------------------ //
#define Ns 10
#define Nca 10
#define Nsca 100


int ns = 0;
int nca = 0;
int nsca = 0;
double p = 0.125;
uint32_t Ws = 0;
// ----------------------- custom ------------------------ //

// SlowStart (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
// {
//   NS_LOG_FUNCTION (this << tcb << segmentsAcked);

//   if (segmentsAcked >= 1)
//     {
//       uint32_t sndCwnd = tcb->m_cWnd;
//       tcb->m_cWnd = std::min ((sndCwnd + (segmentsAcked * tcb->m_segmentSize)), (uint32_t)tcb->m_ssThresh);
//       NS_LOG_INFO ("In SlowStart, updated to cwnd " << tcb->m_cWnd << " ssthresh " << tcb->m_ssThresh);
//       return segmentsAcked - ((tcb->m_cWnd - sndCwnd) / tcb->m_segmentSize);
//     }

//   return 0;
// }

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TcpVegasW");
NS_OBJECT_ENSURE_REGISTERED (TcpVegasW);

TypeId
TcpVegasW::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpVegasW")
    .SetParent<TcpNewReno> ()
    .AddConstructor<TcpVegasW> ()
    .SetGroupName ("Internet")
    .AddAttribute ("Alpha", "Lower bound of packets in network",
                   UintegerValue (2),
                   MakeUintegerAccessor (&TcpVegasW::m_alpha),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Beta", "Upper bound of packets in network",
                   UintegerValue (4),
                   MakeUintegerAccessor (&TcpVegasW::m_beta),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Gamma", "Limit on increase",
                   UintegerValue (1),
                   MakeUintegerAccessor (&TcpVegasW::m_gamma),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}

TcpVegasW::TcpVegasW (void)
  : TcpNewReno (),
    m_alpha (2),
    m_beta (4),
    m_gamma (1),
    m_baseRtt (Time::Max ()),
    m_minRtt (Time::Max ()),
    m_cntRtt (0),
    m_doingVegasNow (true),
    m_begSndNxt (0)
{
  NS_LOG_FUNCTION (this);
}

TcpVegasW::TcpVegasW (const TcpVegasW& sock)
  : TcpNewReno (sock),
    m_alpha (sock.m_alpha),
    m_beta (sock.m_beta),
    m_gamma (sock.m_gamma),
    m_baseRtt (sock.m_baseRtt),
    m_minRtt (sock.m_minRtt),
    m_cntRtt (sock.m_cntRtt),
    m_doingVegasNow (true),
    m_begSndNxt (0)
{
  NS_LOG_FUNCTION (this);
}

TcpVegasW::~TcpVegasW (void)
{
  NS_LOG_FUNCTION (this);
}

Ptr<TcpCongestionOps>
TcpVegasW::Fork (void)
{
  return CopyObject<TcpVegasW> (this);
}

void
TcpVegasW::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                     const Time& rtt)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked << rtt);

  if (rtt.IsZero ())
    {
      return;
    }

  m_minRtt = std::min (m_minRtt, rtt);
  NS_LOG_DEBUG ("Updated m_minRtt = " << m_minRtt);

  m_baseRtt = std::min (m_baseRtt, rtt);
  NS_LOG_DEBUG ("Updated m_baseRtt = " << m_baseRtt);

  // Update RTT counter
  m_cntRtt++;
  NS_LOG_DEBUG ("Updated m_cntRtt = " << m_cntRtt);
}

void
TcpVegasW::EnableVegas (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);

  m_doingVegasNow = true;
  m_begSndNxt = tcb->m_nextTxSequence;
  m_cntRtt = 0;
  m_minRtt = Time::Max ();
}

void
TcpVegasW::DisableVegas ()
{
  NS_LOG_FUNCTION (this);

  m_doingVegasNow = false;
}

void
TcpVegasW::CongestionStateSet (Ptr<TcpSocketState> tcb,
                              const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this << tcb << newState);
  EnableVegas(tcb);
  // if (newState == TcpSocketState::CA_OPEN)
  //   {
  //     EnableVegas (tcb);
  //   }
  // else
  //   {
  //     DisableVegas ();
  //   }
}

void
TcpVegasW::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (!m_doingVegasNow)
    {
      // If Vegas is not on, we follow NewReno algorithm
      NS_LOG_LOGIC ("Vegas is not turned on, we follow NewReno algorithm.");
      IncreaseWindowReno (tcb, segmentsAcked);
      return;
    }

  if (tcb->m_lastAckedSeq >= m_begSndNxt)
    { // A Vegas cycle has finished, we do Vegas cwnd adjustment every RTT.

      NS_LOG_LOGIC ("A Vegas cycle has finished, we adjust cwnd once per RTT.");
      // Save the current right edge for next Vegas cycle
      m_begSndNxt = tcb->m_nextTxSequence;

      /*
       * We perform Vegas calculations only if we got enough RTT samples to
       * insure that at least 1 of those samples wasn't from a delayed ACK.
       */
      if (m_cntRtt <= 2)
        {  // We do not have enough RTT samples, so we should behave like Reno
          NS_LOG_LOGIC ("We do not have enough RTT samples to do Vegas, so we behave like NewReno.");
          IncreaseWindowReno (tcb, segmentsAcked);
        }
      else
        {
          NS_LOG_LOGIC ("We have enough RTT samples to perform Vegas calculations");
          /*
           * We have enough RTT samples to perform Vegas algorithm.
           * Now we need to determine if cwnd should be increased or decreased
           * based on the calculated difference between the expected rate and actual sending
           * rate and the predefined thresholds (alpha, beta, and gamma).
           */
          uint32_t diff;
          uint32_t targetCwnd;
          uint32_t segCwnd = tcb->GetCwndInSegments ();

          /*
           * Calculate the cwnd we should have. baseRtt is the minimum RTT
           * per-connection, minRtt is the minimum RTT in this window
           *
           * little trick:
           * desidered throughput is currentCwnd * baseRtt
           * target cwnd is throughput / minRtt
           */
          double tmp = m_baseRtt.GetSeconds () / m_minRtt.GetSeconds ();
          targetCwnd = static_cast<uint32_t> (segCwnd * tmp);
          NS_LOG_DEBUG ("Calculated targetCwnd = " << targetCwnd);
          NS_ASSERT (segCwnd >= targetCwnd); // implies baseRtt <= minRtt

          /*
           * Calculate the difference between the expected cWnd and
           * the actual cWnd
           */
          diff = segCwnd - targetCwnd;
          NS_LOG_DEBUG ("Calculated diff = " << diff);

          if (diff > m_gamma && (tcb->m_cWnd < tcb->m_ssThresh))
            {
              /*
               * We are going too fast. We need to slow down and change from
               * slow-start to linear increase/decrease mode by setting cwnd
               * to target cwnd. We add 1 because of the integer truncation.
               */
              NS_LOG_LOGIC ("We are going too fast. We need to slow down and "
                            "change to linear increase/decrease mode.");
              
              // segCwnd = std::min (segCwnd, targetCwnd + 1);
              segCwnd = segCwnd * (1-p);                                              // custom
              tcb->m_cWnd = segCwnd * tcb->m_segmentSize;
              
              tcb->m_ssThresh = tcb->m_cWnd.Get() - 1;
              
              NS_LOG_DEBUG ("Updated cwnd = " << tcb->m_cWnd <<" ssthresh=" << tcb->m_ssThresh);
            }
          else if (tcb->m_cWnd < tcb->m_ssThresh)                                   // diff < gamma
            {
                 // Slow start mode
              NS_LOG_LOGIC ("We are in slow start and diff < m_gamma, so we "
                            "follow NewReno slow start");

              SlowStart (tcb, segmentsAcked);
            }
          else
            {     // Linear increase/decrease mode
              NS_LOG_LOGIC ("We are in linear increase/decrease mode");
              if (diff > m_beta)
                {
                  // We are going too fast, so we slow down
                  NS_LOG_LOGIC ("We are going too fast, so we slow down by decrementing cwnd");
                  segCwnd--;
                  tcb->m_cWnd = segCwnd * tcb->m_segmentSize;  
                  NS_LOG_DEBUG ("Updated cwnd = " << tcb->m_cWnd <<" ssthresh=" << tcb->m_ssThresh);

                  nsca = 0;
                }
              else if (diff < m_alpha)
                {
                  // We are going too slow (having too little data in the network),
                  // so we speed up.
                  NS_LOG_LOGIC ("We are going too slow, so we speed up by incrementing cwnd");
                  
                  if(nca <= Nca)
                  {
                    nca++;
                  }
                  else
                  {
                    segCwnd++;
                    tcb->m_cWnd = segCwnd * tcb->m_segmentSize;
                    nca = 0;
                  }
                  
                  NS_LOG_DEBUG ("Updated cwnd = " << tcb->m_cWnd <<" ssthresh=" << tcb->m_ssThresh);
                }
              else
                { // alpha < diff < beta
                  // We are going at the right speed
                  NS_LOG_LOGIC ("We are sending at the right speed");

                  if(tcb->m_cWnd.Get() > Ws)
                  {
                    nsca = 0;
                    Ws = tcb->m_cWnd.Get();
                  }
                  nsca++;
                  if(nsca > Nsca)
                  {
                    tcb->m_ssThresh = tcb->m_cWnd;
                    nsca = 0;
                  }
                }
            }
          // tcb->m_ssThresh = std::max (tcb->m_ssThresh, 3 * tcb->m_cWnd / 4);
          // NS_LOG_UNCOND ("Updated ssThresh = " << tcb->m_ssThresh);
          // NS_LOG_DEBUG ("Updated ssThresh = " << tcb->m_ssThresh);
        }

      // Reset cntRtt & minRtt every RTT
      m_cntRtt = 0;
      m_minRtt = Time::Max ();
    }
  else if (tcb->m_cWnd < tcb->m_ssThresh)
    {
      // TcpNewReno::SlowStart (tcb, segmentsAcked);
      SlowStart (tcb, segmentsAcked);
    }
}

std::string
TcpVegasW::GetName () const
{
  return "TcpVegasW";
}

uint32_t
TcpVegasW::GetSsThresh (Ptr<const TcpSocketState> tcb,
                       uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this << tcb << bytesInFlight);
  return std::max (std::min (tcb->m_ssThresh.Get (), tcb->m_cWnd.Get () - tcb->m_segmentSize), 2 * tcb->m_segmentSize);
  // return tcb->m_ssThresh.Get () - 1;
}



// ----------------- functions of TcpNewReno ------------------- //

// ********** Slower Start ********** //
uint32_t
TcpVegasW::SlowStart (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (segmentsAcked >= 1)
    {
      if(ns > Ns)
      {
        tcb->m_cWnd += tcb->m_segmentSize;
        NS_LOG_INFO ("In SlowStart, updated to cwnd " << tcb->m_cWnd << " ssthresh " << tcb->m_ssThresh);
        ns = 0;
      }
      else
      {
        ns++;
      }
      return segmentsAcked - 1;
    }

  return 0;
}


/**
 * \brief NewReno congestion avoidance
 *
 * During congestion avoidance, cwnd is incremented by roughly 1 full-sized
 * segment per round-trip time (RTT).
 *
 * \param tcb internal congestion state
 * \param segmentsAcked count of segments acked
 */
void
TcpVegasW::CongestionAvoidanceReno (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (segmentsAcked > 0)
    {
      double adder = static_cast<double> (tcb->m_segmentSize * tcb->m_segmentSize) / tcb->m_cWnd.Get ();
      adder = std::max (1.0, adder);
      tcb->m_cWnd += static_cast<uint32_t> (adder);
      NS_LOG_INFO ("In CongAvoid, updated to cwnd " << tcb->m_cWnd <<
                   " ssthresh " << tcb->m_ssThresh);
    }
}

/**
 * \brief Try to increase the cWnd following the NewReno specification
 *
 * \see SlowStart
 * \see CongestionAvoidance
 *
 * \param tcb internal congestion state
 * \param segmentsAcked count of segments acked
 */
void
TcpVegasW::IncreaseWindowReno (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (tcb->m_cWnd < tcb->m_ssThresh)
    {
      segmentsAcked = SlowStart (tcb, segmentsAcked);
    }

  if (tcb->m_cWnd >= tcb->m_ssThresh)
    {
      CongestionAvoidanceReno (tcb, segmentsAcked);
    }

  /* At this point, we could have segmentsAcked != 0. This because RFC says
   * that in slow start, we should increase cWnd by min (N, SMSS); if in
   * slow start we receive a cumulative ACK, it counts only for 1 SMSS of
   * increase, wasting the others.
   *
   * // Incorrect assert, I am sorry
   * NS_ASSERT (segmentsAcked == 0);
   */
}

} // namespace ns3
