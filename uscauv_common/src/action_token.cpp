#include <uscauv_common/action_token.h>

std::function<bool()> quickdev::action_token::make_term_criteria( SimpleActionToken token )
{
    return std::bind( &ActionTokenStorage<boost::thread>::done, token.getStorage().get() );
}
