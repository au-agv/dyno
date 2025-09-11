/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( D ) ( Y ) ( N ) ( O )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +              DYNO: Ground Vehicle Dynamics Validation Toolkit             +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <chrono>
#include <ctime>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <string>
#include <vector>

#include <chrono_vehicle/ChDriver.h>
#include <chrono_vehicle/ChTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <spdlog/spdlog.h>

namespace DYNO {
namespace Serialization {

class Serializer {
   public:
    Serializer(std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
               std::shared_ptr<chrono::vehicle::ChDriver> driver,
               std::shared_ptr<chrono::vehicle::ChTerrain> terrain);

    std::string GetFilename();

    void SetFilename(std::string filename);

    bool AddTimestamp();

    void AddTimestamp(bool add_timestamp);

    bool UseNameGenerator();

    void UseNameGenerator(bool use_name_generator);

    void WriteToDisk();

    virtual void Initialize();

    void CreateSubfolder(const bool& create_subfolder) {
        create_subfolder_ = create_subfolder;
    }

    virtual void Save(double time) = 0;

    virtual void Dump() = 0;

    virtual void AddResult(const std::string& key, const double& value) = 0;

    virtual void AddMetadata(const std::string& key, const double& value) = 0;

    virtual void AddMetadata(const std::string& key,
                             const std::string& value) = 0;

    virtual void AddTriggers(const std::string& key,
                             const std::vector<double>& coordinates,
                             const std::vector<double>& quantities) = 0;

   protected:
    std::unordered_map<std::string, std::vector<double>> map_;

    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle_;

    std::shared_ptr<chrono::vehicle::ChDriver> driver_;

    std::shared_ptr<chrono::vehicle::ChTerrain> terrain_;

    void InitializePath();

    std::string path_;

    std::string filename_ = "unnamed";

    std::string file_path_;

    void InitializeTimestamp();

    bool add_timestamp_ = false;

    std::string date_;

    std::string time_;

    void InitializeFriendlyName();

    bool use_name_generator_ = false;

    int random_name_index_ = -1;

    std::vector<std::string> names_ = std::vector<std::string>{
        "agnesi",     "albattani",     "allen",        "almeida",
        "antonelli",  "archimedes",    "ardinghelli",  "aryabhata",
        "austin",     "babbage",       "banach",       "banzai",
        "bardeen",    "bartik",        "bassi",        "beaver",
        "bell",       "benz",          "bhabha",       "bhaskara",
        "black",      "blackburn",     "blackwell",    "bohr",
        "booth",      "borg",          "bose",         "bouman",
        "boyd",       "brahmagupta",   "brattain",     "brown",
        "buck",       "burnell",       "cannon",       "carson",
        "cartwright", "carver",        "cerf",         "chandrasekhar",
        "chaplygin",  "chatelet",      "chatterjee",   "chaum",
        "chebyshev",  "clarke",        "cohen",        "colden",
        "cori",       "cray",          "curie",        "curran",
        "darwin",     "davinci",       "dewdney",      "dhawan",
        "diffie",     "dijkstra",      "dirac",        "driscoll",
        "dubinsky",   "easley",        "edison",       "einstein",
        "elbakyan",   "elgamal",       "elion",        "ellis",
        "engelbart",  "euclid",        "euler",        "faraday",
        "feistel",    "fermat",        "fermi",        "feynman",
        "franklin",   "gagarin",       "galileo",      "galois",
        "ganguly",    "gates",         "gauss",        "germain",
        "goldberg",   "goldstine",     "goldwasser",   "golick",
        "goodall",    "gould",         "greider",      "grothendieck",
        "haibt",      "hamilton",      "haslett",      "hawking",
        "heisenberg", "hellman",       "hermann",      "herschel",
        "hertz",      "heyrovsky",     "hodgkin",      "hofstadter",
        "hoover",     "hopper",        "hugle",        "hypatia",
        "ishizaka",   "jackson",       "jang",         "jemison",
        "jennings",   "jepsen",        "johnson",      "joliot",
        "jones",      "kalam",         "kapitsa",      "kare",
        "keldysh",    "keller",        "kepler",       "khayyam",
        "khorana",    "kilby",         "kirch",        "knuth",
        "kowalevski", "lalande",       "lamarr",       "lamport",
        "leakey",     "leavitt",       "lederberg",    "lehmann",
        "lewin",      "lichterman",    "liskov",       "lovelace",
        "lumiere",    "mahavira",      "margulis",     "matsumoto",
        "maxwell",    "mayer",         "mccarthy",     "mcclintock",
        "mclaren",    "mclean",        "mcnulty",      "meitner",
        "mendel",     "mendeleev",     "meninsky",     "merkle",
        "mestorf",    "mirzakhani",    "montalcini",   "moore",
        "morse",      "moser",         "murdock",      "napier",
        "nash",       "neumann",       "newton",       "nightingale",
        "nobel",      "noether",       "northcutt",    "noyce",
        "panini",     "pare",          "pascal",       "pasteur",
        "payne",      "perlman",       "pike",         "poincare",
        "poitras",    "proskuriakova", "ptolemy",      "raman",
        "ramanujan",  "rhodes",        "ride",         "ritchie",
        "robinson",   "roentgen",      "rosalind",     "rubin",
        "saha",       "sammet",        "sanderson",    "satoshi",
        "shamir",     "shannon",       "shaw",         "shirley",
        "shockley",   "shtern",        "sinoussi",     "snyder",
        "solomon",    "spence",        "stonebraker",  "sutherland",
        "swanson",    "swartz",        "swirles",      "taussig",
        "tesla",      "tharp",         "thompson",     "torvalds",
        "tu",         "turing",        "varahamihira", "vaughan",
        "villani",    "visvesvaraya",  "volhard",      "wescoff",
        "wilbur",     "wiles",         "williams",     "williamson",
        "wilson",     "wing",          "wozniak",      "wright",
        "wu",         "yalow",         "yonath",       "zhukovsky"};

    int random_adjective_index_ = -1;

    std::vector<std::string> adjectives_ = std::vector<std::string>{
        "admiring",    "adoring",       "affectionate", "agitated",
        "amazing",     "angry",         "awesome",      "beautiful",
        "blissful",    "bold",          "boring",       "brave",
        "busy",        "charming",      "clever",       "compassionate",
        "competent",   "condescending", "confident",    "cool",
        "cranky",      "crazy",         "dazzling",     "determined",
        "distracted",  "dreamy",        "eager",        "ecstatic",
        "elastic",     "elated",        "elegant",      "eloquent",
        "epic",        "exciting",      "fervent",      "festive",
        "flamboyant",  "focused",       "friendly",     "frosty",
        "funny",       "gallant",       "gifted",       "goofy",
        "gracious",    "great",         "happy",        "hardcore",
        "heuristic",   "hopeful",       "hungry",       "infallible",
        "inspiring",   "intelligent",   "interesting",  "jolly",
        "jovial",      "keen",          "kind",         "laughing",
        "loving",      "lucid",         "magical",      "modest",
        "musing",      "mystifying",    "naughty",      "nervous",
        "nice",        "nifty",         "nostalgic",    "objective",
        "optimistic",  "peaceful",      "pedantic",     "pensive",
        "practical",   "priceless",     "quirky",       "quizzical",
        "recursing",   "relaxed",       "reverent",     "romantic",
        "sad",         "serene",        "sharp",        "silly",
        "sleepy",      "stoic",         "strange",      "stupefied",
        "suspicious",  "sweet",         "tender",       "thirsty",
        "trusting",    "unruffled",     "upbeat",       "vibrant",
        "vigilant",    "vigorous",      "wizardly",     "wonderful",
        "xenodochial", "youthful",      "zealous",      "zen"};

   private:
    bool create_subfolder_ = false;
};

}  // namespace Serialization
}  // namespace DYNO
