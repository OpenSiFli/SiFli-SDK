# JSON
A SAX like JSON on-the-fly parser with no memory allocation while parsing.
It's inspired by JSMM for working principle but is completely rewritten for C++ and support partial parsing.
 
It was developped to fit a memory & flash constrained microcontroller, so many design choice are oriented toward saving binary size and memory.

## Drawbacks
It's designed for stream parsing (that is, there is no limit to parse end of "logical end").
It checks the JSON syntax, but accept few invalid constructions:

 - `{}{}` : it'll stop on the first object declaration but not return an error since it hasn't seen the other object yet.
 - `[123.E232-23++34.24...2424]`: Since number are not converted, this will be accepted as an array containing a number. You'll notice the error if you try to convert the number (if you either do that).
 
By default, it does not modify its input and work-in-place.
It does not convert/unescape strings unless you `#define UnescapeJSON` (in that case, the input text is modified to store 0 at strings' ends).
Even in that case, it does not convert unicode escaped chars (\uABCD) to UTF-8 (usually, this is not required since such strings can be returned untouched).
It does not convert textual number to native number (you can use atoi/atof like functions yourself).

## Advantages

It does not allocate any memory. It's deterministic.
It tags all objects and arrays with a unique identifier (useful for partial parsing, see below).
Such identifier is limited to 4095 possible values (before wrap around).


It should be very fast to parse and with minimal code size.
(On my machine complete code compiles to less than 2.5kB)

It's able to resume parsing on a new received buffer so you don't need to store the complete JSON stream in memory at anytime.

It accepts read only input buffer, so it can work on memory mapped flash (on embedded system), without a heap.
This usually saves a lot of precious heap space required by competing JSON parser.

## Partial parsing
It also supports (optional) partial parsing.
This allows calling the parse method with an incomplete buffer, start working with the partial JSON object, then, when more data is ready, continue parsing from where it lefts off.
This feature works mostly in zero-copy mode (it works on the input buffer, with no allocation).
Typically, that's good if you're receiving data from a socket and don't have enough buffers/memory to accumulate the complete JSON text before parsing.
 

Partial parsing is a feature that's almost as big as the parse method in binary so if you don't need it you either `#define SkipJSONPartialParsing`, or build with `-ffunction-sections -fdata-sections -Wl,-gc-sections` flags so the linker will garbage collect the function (this should be the default for embedded code anyway).
This removes 800 bytes from the compiled binary on my machine.
The function is logically written so it resumes after the initial parse function returned Starving error.
 

A partial parsing rewrites the token stream so the object hierarchy to the last values (with key if available) found is kept valid but all previous values (opt. keys) are removed. For ease of use reasons, if the parsing stopped in the middle of a a key/value pair,
the token stream will contains the last key the (interrupted and resumed) value refers to.
When processing the token stream, you are ensured to always have a key before a value in an object.
Since object and arrays are identified, you can also figure out in which object the key/value refers to by remembering the container's ID. Try to accumulate as much data as possible before calling partialParsing, since this rewriting is computation expensive, so avoid to do that for each byte received.

## SAX parsing
In this case, you don't even need to allocate a token array at all. You'll call `JSON::parseOne` in a loop, each iteration mutating the given token, until it returns `Finished`.

Using SAX parsing is like receiving parsing events (like entering an object, receiving a key, a value, etc...). 
Please refer to `SAXState` enumeration for the different event's states.

The downside however, is that you don't know the number of elements at all (it's not tracked anymore) and the hierarchy of objects (you have to
keep track of this by yourself). So the `token.parent` is useless, and it was re-used to store the `SAXState` event type instead.
You can call `getCurrentContainerCount` to parse the buffer upon entering a container, if absolutely required.

Also, you need to maintain a stack of the parent object's positions in order for the parser to validate the validity of the input JSON.
Please refer to `JSONSaxTest.cpp` for an example usage of this API.



## Implementation limits
The default implementation limits input JSON size to signed 16 bits (32768 bytes), and less than 4095 embedded objects/arrays.
If you intend to parse more than that, you'll need to use to a larger **signed** type for the template parameter.
The Token's size is, by default, 8 bytes long. Using signed 32 bits int for the IndexType will double it's size.
The parser memory size is 10 bytes by default. It requires less than a hundred bytes of stack space.
The code is very clean and small (less than 300 lines of code for the definition)


Because I got the question, you can not write a JSON stream with any parser and even more with this one.
Writing small JSON is trivial, but it's not the subject of this class.

## Dependencies
There is no dependency on STL, and no exceptions either.
Only `memmove` is used in partialParsing for ensuring the previous key is present in the new stream to parse.

## Requirements
Only `JSON.hpp` and `JSON.tpp` is required, the other files are used for testing the code. The code is using a MIT license.

## Speed
This parser is not optimized for speed. There is no SIMD parsing like some competing parsers. 
However, since its API is very simple, the client code is usually a lot smaller and easier to get right.
It's faster than JSMM, but probably slower than RapidJSON. 
