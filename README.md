## How do I use types with generic parameters in RTIC Resources?

I asked how to use the
[ws2812-pio](https://github.com/ithinuel/ws2812-pio-rs/blob/main/src/lib.rs)
crate from RTIC on the Matrix channel and received some advice that was enough
to get me to this state.

> ... name and type are converted into a static mut variable, so the resources [must
> be] fully specified and 'static lifetimes.

The 'static hint was the key to the second generic parameter, but this crate
seems to do quite a lot of work to construct the first parameter and I have been
unable to find a satisfactory type.
