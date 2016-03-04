/*
 *  music_cont_out_proxy.cpp
 *
 *  This file is part of NEST.
 *
 *  Copyright (C) 2004 The NEST Initiative
 *
 *  NEST is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  NEST is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NEST.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "config.h"

#ifdef HAVE_MUSIC

#include "music_cont_out_proxy.h"
#include "network.h"
#include "dict.h"
#include "integerdatum.h"
#include "doubledatum.h"
#include "dictutils.h"
#include "arraydatum.h"

#include <numeric>

/* ----------------------------------------------------------------
 * Default constructors defining default parameters and state
 * ---------------------------------------------------------------- */

nest::music_cont_out_proxy::Parameters_::Parameters_()
  : port_name_( "cont_out" )
  , interval_( Time::ms( 1.0 ) )
  , record_from_()
{
}

nest::music_cont_out_proxy::Parameters_::Parameters_( const Parameters_& p )
  : port_name_( p.port_name_ )
  , interval_( p.interval_ )
  , record_from_( p.record_from_ )
{
  interval_.calibrate();
}

nest::music_cont_out_proxy::State_::State_()
  : published_( false )
  , port_width_( -1 )
  , max_buffered_( -1 )
{
}

nest::music_cont_out_proxy::Buffers_::Buffers_()
  : has_targets_( false )
{
}

/* ----------------------------------------------------------------
 * Parameter extraction and manipulation functions
 * ---------------------------------------------------------------- */

void
nest::music_cont_out_proxy::Parameters_::get( DictionaryDatum& d ) const
{
  ( *d )[ names::port_name ] = port_name_;
  ( *d )[ names::interval ] = interval_.get_ms();
  ArrayDatum ad;
  for ( size_t j = 0; j < record_from_.size(); ++j )
    ad.push_back( LiteralDatum( record_from_[ j ] ) );
  ( *d )[ names::record_from ] = ad;
}

void
nest::music_cont_out_proxy::Parameters_::set( const DictionaryDatum& d, const State_& s, const Buffers_& b )
{
  // TODO: This is not possible, as P_ does not know about get_name()
  //  if(d->known(names::port_name) && s.published_)
  //    throw MUSICPortAlreadyPublished(get_name(), P_.port_name_);

  if ( !s.published_ )
    updateValue< string >( d, names::port_name, port_name_ );

  if ( b.has_targets_ && ( d->known( names::interval ) || d->known( names::record_from ) ) )
    throw BadProperty(
      "The recording interval and the list of properties to record "
      "cannot be changed after the multimeter has been connected to "
      "nodes." );

  double_t v;
  if ( updateValue< double_t >( d, names::interval, v ) )
  {
    if ( Time( Time::ms( v ) ) < Time::get_resolution() )
      throw BadProperty(
        "The sampling interval must be at least as long "
        "as the simulation resolution." );

    // see if we can represent interval as multiple of step
    interval_ = Time::step( Time( Time::ms( v ) ).get_steps() );
    if ( std::abs( 1 - interval_.get_ms() / v ) > 10 * std::numeric_limits< double >::epsilon() )
      throw BadProperty(
        "The sampling interval must be a multiple of "
        "the simulation resolution" );
  }

  // extract data
  if ( d->known( names::record_from ) )
  {
    record_from_.clear();

    ArrayDatum ad = getValue< ArrayDatum >( d, names::record_from );
    for ( Token* t = ad.begin(); t != ad.end(); ++t )
      record_from_.push_back( Name( getValue< std::string >( *t ) ) );
  }
}

void
nest::music_cont_out_proxy::State_::get( DictionaryDatum& d ) const
{
  ( *d )[ names::published ] = published_;
  ( *d )[ names::port_width ] = port_width_;
  ( *d )[ names::max_buffered ] = max_buffered_;
}

void
nest::music_cont_out_proxy::State_::set( const DictionaryDatum&, const Parameters_& )
{
}


/* ----------------------------------------------------------------
 * Default and copy constructor for node
 * ---------------------------------------------------------------- */

nest::music_cont_out_proxy::music_cont_out_proxy()
  : Node()
  //, device_( *this, RecordingDevice::MULTIMETER, "dat", true, true )
  , P_()
  , S_()
  , V_()
  , B_()
{
}

nest::music_cont_out_proxy::music_cont_out_proxy( const music_cont_out_proxy& n )
  : Node( n )
  // , device_( *this, n.device_ )
  , P_( n.P_ )
  , S_( n.S_ )
  , V_( n.V_ )
  , B_( n.B_ )
{
}

nest::music_cont_out_proxy::~music_cont_out_proxy()
{
  if ( S_.published_ )
  {
    delete V_.MP_;
    delete V_.music_perm_ind_;
  }
}

void
nest::music_cont_out_proxy::init_state_( const Node& /* np */ )
{
  // const Multimeter& asd = dynamic_cast< const Multimeter& >( np );
  // device_.init_state( asd.device_ );
  B_.data_.clear();
}

void
nest::music_cont_out_proxy::init_buffers_()
{
  // device_.init_buffers();
}

void nest::music_cont_out_proxy::finalize()
{
  // device_.finalize();
}


nest::port nest::music_cont_out_proxy::send_test_event( Node& target, rport receptor_type, synindex, bool )
{

  if ( !S_.published_ )
    V_.index_map_.push_back( static_cast< int >( receptor_type ) );
  else
    throw MUSICPortAlreadyPublished( get_name(), P_.port_name_ );

  DataLoggingRequest e( P_.interval_, P_.record_from_ );
  e.set_sender( *this );
  port p = target.handles_test_event( e, receptor_type );
  if ( p != invalid_port_ and not is_model_prototype() )
    B_.has_targets_ = true;
  return p;
}



//OK
void
nest::music_cont_out_proxy::calibrate()
{
  // device_.calibrate();
  V_.new_request_ = false;
  V_.current_request_data_start_ = 0;

  // only publish the output port once,
  if ( !S_.published_ )
  {
    MUSIC::Setup* s = nest::Communicator::get_music_setup();
    if ( s == 0 )
      throw MUSICSimulationHasRun( get_name() );

    V_.MP_ = s->publishContOutput( P_.port_name_ );

    if ( !V_.MP_->isConnected() )
      throw MUSICPortUnconnected( get_name(), P_.port_name_ );

    if ( !V_.MP_->hasWidth() )
      throw MUSICPortHasNoWidth( get_name(), P_.port_name_ );

    S_.port_width_ = V_.MP_->width();
    const size_t doubles_per_port = P_.record_from_.size();

    // Allocate memory
    B_.data_ = std::vector< double >( doubles_per_port * S_.port_width_ );

    // Check if any port is out of bounds
    std::vector< MUSIC::GlobalIndex >::const_iterator it;
    for ( it = V_.index_map_.begin(); it != V_.index_map_.end(); ++it )
      if ( *it > S_.port_width_ )
        throw UnknownReceptorType( *it, get_name() );

    // The permutation index map, contains global_index[local_index]
    V_.music_perm_ind_ = new MUSIC::PermutationIndex( &V_.index_map_.front(), V_.index_map_.size() );

    // New MPI datatype which is a compound of multiple double values
    MPI_Datatype multi_double_type;
    MPI_Type_contiguous(doubles_per_port, MPI::DOUBLE, &multi_double_type);

    // Setup an array map
    MUSIC::ArrayData dmap ( static_cast< void* >( &( B_.data_.front() ) ), multi_double_type, V_.music_perm_ind_ );

    if( S_.max_buffered_ > 0 )
        V_.MP_->map( &dmap, S_.max_buffered_ );
    else
        V_.MP_->map( &dmap );

    // check, if there are connections to receiver ports, which are
    // beyond the width of the port
    //std::vector< MUSIC::GlobalIndex >::const_iterator it;
    //for ( it = V_.index_map_.begin(); it != V_.index_map_.end(); ++it )
    //  if ( *it > S_.port_width_ )
    //    throw UnknownReceptorType( *it, get_name() );

    // The permutation index map, contains global_index[local_index]
    //V_.music_perm_ind_ =
     // new MUSIC::PermutationIndex( &V_.index_map_.front(), V_.index_map_.size() );

    // we identify channels by global indices within NEST
    //V_.MP_->map( V_.music_perm_ind_, MUSIC::Index::GLOBAL );

    S_.published_ = true;

    std::string msg = String::compose(
      "Mapping MUSIC output port '%1' with width=%2.", P_.port_name_, S_.port_width_ );
    net_->message( SLIInterpreter::M_INFO, "MusicEventHandler::publish_port()", msg.c_str() );
  }
}

//TODO
void
nest::music_cont_out_proxy::get_status( DictionaryDatum& d ) const
{

  // get the data from the device
  // device_.get_status( d );

  // if we are the device on thread 0, also get the data from the
  // siblings on other threads
  if ( get_thread() == 0 )
  {
    const SiblingContainer* siblings = network()->get_thread_siblings( get_gid() );
    std::vector< Node* >::const_iterator sibling;
    for ( sibling = siblings->begin() + 1; sibling != siblings->end(); ++sibling )
      ( *sibling )->get_status( d );
  }

  P_.get( d );
  S_.get( d );
}

//TODO
void nest::music_cont_out_proxy::set_status( const DictionaryDatum& d )
{
  Parameters_ ptmp = P_; // temporary copy in case of errors
  ptmp.set( d, S_, B_ );     // throws if BadProperty

  State_ stmp = S_;
  stmp.set( d, P_ ); // throws if BadProperty

  // if we get here, temporaries contain consistent set of properties
  P_ = ptmp;
  S_ = stmp;
}

void nest::music_cont_out_proxy::update( Time const& origin, const long_t from, const long_t )
{
  /* There is nothing to request during the first time slice.
     For each subsequent slice, we collect all data generated during the previous
     slice if we are called at the beginning of the slice. Otherwise, we do nothing.
   */
  if ( origin.get_steps() == 0 || from != 0 )
    return;

  // We send a request to each of our targets.
  // The target then immediately returns a DataLoggingReply event,
  // which is caught by multimeter::handle(), which in turn
  // ensures that the event is recorded.
  // handle() has access to request_, so it knows what we asked for.
  //
  // Provided we are recording anything, V_.new_request_ is set to true. This informs
  // handle() that the first incoming DataLoggingReply is for a new time slice, so that
  // the data from that first Reply must be pushed back; all following Reply data is
  // then added.
  //
  // Note that not all nodes receiving the request will necessarily answer.
  V_.new_request_ = B_.has_targets_ && !P_.record_from_.empty(); // no targets, no request
  DataLoggingRequest req;
  network()->send( *this, req );
}

void
nest::music_cont_out_proxy::handle( DataLoggingReply& reply )
{
  // easy access to relevant information
  DataLoggingReply::Container const& info = reply.get_info();

  const index receiver_port = reply.get_rport();
  const size_t record_width = P_.record_from_.size();
  // record all data, time point by time point
  for ( size_t j = 0; j < info.size(); ++j )
  {
    if ( info[ j ].timestamp.is_finite() )
    {
        // S_.data_.push_back( info[ info.size()-1 ].data );
        const DataLoggingReply::DataItem item = info[ j ].data;
        const std::vector<double>::iterator port_addr = B_.data_.begin() + (receiver_port * record_width);
         
    }
    // store stamp for current data set in event for logging
    // reply.set_stamp( info[ j ].timestamp );

    // record sender and time information; in accumulator mode only for first Reply in slice
    // device_.record_event( reply, false ); // false: more data to come

    //  print_value_( info[ j ].data );

    // S_.data_.push_back( info[ j ].timestamp );
  }

  V_.new_request_ = false; // correct either we are done with the first reply or any later one
}

#endif
