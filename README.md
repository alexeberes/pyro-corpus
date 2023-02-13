# pyro-corp: Simulated Virtual Robot Body Evolution

Simulating random robot body plans using a recursive method.

To run, clone and run `python main.py` in the terminal.

## Motivation

Natural evolution features mutations and adaptations to not only the brains of organisms, but also their bodies.
Hence, simulated evolution should also seek to optimize virtual creatures morphologies in addition to their intelligence systems.

Additionally, saving the entire body plan of an organism is inefficient, especially since parts are often replicated.
Consider biological organisms.
Each one is made of up collections of myriads of almost identical cells.
The organism does not explicitly store each and every cell separately.
Rather, there is a large set of instructions that is repeated several times.

Towards this goal, here I present a recursive method of generating simulated robot body plans.
To demonstrate the capabilities of the method, I generated and simulated 5 semi-random robot body plans, seen in the video here.

[![recursive body generation video](https://img.youtube.com/vi/lkgSwKMEpzM/0.jpg)](https://www.youtube.com/watch?v=lkgSwKMEpzM)

## The method itself

The robot bodies were simulated using the [pyrosim](https://github.com/jbongard/pyrosim) library.

The recursive body generation method takes in a body plan coded in the following format:

`(BodyPart, number_of_repetitions, NextInstruction)`.

For the semi-random robots generated in the video, the body plan given was (in pseudocode)

`(RandomBodyPart, r1, (RandomSensorPart, r2, (RandomBodyPart, r3, (RandomSensorPart, r4, (RandomBodyPart, r5, (RandomSensorPiece, r6, None))))))`

where `r1` through `r6` are randomly generated integers between 1 and 3.

The recursive method follows the below steps:

0. Verify that a valid instruction has been sent
1. Build the current piece
2. Decrement the repetition counter
3. Check the number of repetitions left; if 0, go to 4.; else fo to X.
4. Check if the next element is None; if so, exit recursion; if not, continue to 5.
5. Recursively call the function with `(NextInstruction.BodPart, NextInstruction.repetitions, NextInstruction.NextInstruction)` as the arguments (i.e., go back to 1.)
6. Recursively call the function with `(BodyPart, number_of_repetitions - 1, NextInstruction)` as the arguments

The robot's brain is built using a second recursive function that follows the same format.
It was necessary to split this into two functions because pyrosim is only able to write to one file type at a time.

## Future work

Future work includes:

* creating a file format to more easily define recursive robot body plans
* move into a 3 dimensional design space instead of only building robot pieces one after the other like a snake

## Special thanks to

* Artificial Life @ Northwestern University taught by [Dr. Sam Kriegman](https://www.mccormick.northwestern.edu/research-faculty/directory/profiles/kriegman-sam.html)
