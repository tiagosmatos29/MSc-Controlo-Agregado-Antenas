/* ============================================================================
 * PROJETO DE DISSERTAÇÃO DE MESTRADO
 * ============================================================================
 * Título:       Agregado de Antenas Multibanda 
 *               para Comunicações sem Fios
 *               em Ambientes Ciríticos Seguras
 * Módulo:       5-Element Beamforming (ULA de 5 elementos)
 * Autores:      Tiago da Silva Matos
 *               Hugo Daniel da Costa Dinis
 * ----------------------------------------------------------------------------
 * Descrição:    Firmware para microcontrolador (C++/Arduino) destinado ao
 *               controlo dinâmico via SPI de phase shifters digitais.
 *               O feedback de fase imprime agora o alvo absoluto esperado
 *               no ecrã do VNA (incluindo a fase de inserção natural do
 *               cabo central de referência).
 *
 *               Beamforming - direciona o lóbulo principal do
 *               Array Factor para uma direção alvo, aplicando
 *               uma progressão de fase linear ao longo do ULA.
 *
 *               Null Steering - coloca um zero exato do
 *               Array Factor numa direção alvo, explorando as posições
 *               naturais dos nulos de um ULA com excitação uniforme.
 * 
 *               Independent Null Steering - Método de Steyskal
 *               para apontar o lobo principal e um nulo em simultâneo 
 *               utilizando apenas perturbações de fase.
 * ============================================================================ */

//////////////////////////////////////////////////////////////
//                    INCLUDES
//////////////////////////////////////////////////////////////

#include <SPI.h>
#include <math.h>

//////////////////////////////////////////////////////////////
//                    CONFIG
//////////////////////////////////////////////////////////////

#define NUM_CHANNELS 4         // 4 phase shifters controlados por SPI
#define ADDR_BASE    2         // endereços SPI reais: 2, 3, 4, 5

#define ARRAY_SIZE   5         // ULA físico: 5 elementos (posições 0..4)
#define REF_ELEMENT_POS 2      // posição 2 (elemento central) = antena de referência, sem phase shifter

// Mapeamento: phase shifter idx (0..3) -> posição física no ULA (0..4)
// idx 0 (addr 2) -> pos 0 | idx 1 (addr 3) -> pos 1
// idx 2 (addr 4) -> pos 3 | idx 3 (addr 5) -> pos 4
// (a posição 2, central, fica de fora por ser a referência sem phase shifter)
const int PS_PHYSICAL_POS[NUM_CHANNELS] = {0, 1, 3, 4};

//////////////////////////////////////////////////////////////
//                    VARIABLES
//////////////////////////////////////////////////////////////

// Beamforming
float element_spacing_lambda = 0.5;      // d = λ/2
float steering_angle = 0.0;

// Null steering
int null_order_default = 1;              // ordem m do nulo por omissão (1 = mais próximo do lobo principal)

// Índice de array: 0 -> addr 2, 1 -> addr 3, 2 -> addr 4, 3 -> addr 5
float channel_phase_deg[NUM_CHANNELS] = {0, 0, 0, 0};
int   channel_phase_bin[NUM_CHANNELS] = {1000, 1000, 1000, 1000};

// Offsets de calibração
float channel_phase_offset[NUM_CHANNELS] = {48.1, 68.6, 11.4, 59.8}; //{-21.84, 1.66, -61.79, -13.74}
float ref_phase = 134.10; //149.44

int sendphase;

// Refresh
unsigned long last_rf_refresh = 0;
const unsigned long REFRESH_INTERVAL_MS = 100;

//////////////////////////////////////////////////////////////
//                    LOOK-UP TABLES
//////////////////////////////////////////////////////////////

// LUT dos bins
int PS_phase_bin_LUT[] = {0,507,252,511,1,253,256,2,257,508,3,254,509,258,255,4,259,510,5,260,6,261,7,262,263,8,9,264,10,265,11,266,12,267,13,268,14,269,15,270,16,17,271,272,18,273,19,274,275,20,21,276,22,277,23,278,279,24,25,280,26,281,27,282,283,28,29,284,30,285,31,286,32,287,33,288,34,289,35,290,36,291,37,292,38,293,39,294,295,40,41,296,42,297,43,298,44,299,45,300,46,301,47,302,303,48,49,304,50,305,51,306,52,307,53,308,54,309,55,310,56,311,57,312,58,313,59,314,60,315,61,316,64,62,317,65,63,320,66,318,321,67,319,322,323,68,69,324,70,325,71,326,327,72,73,328,74,329,75,330,331,76,77,332,78,333,79,334,80,81,335,336,82,337,83,338,339,84,85,340,86,341,87,342,88,343,89,344,90,345,91,346,347,92,93,348,94,349,96,95,97,350,352,98,351,353,99,354,355,100,101,356,102,357,103,358,359,104,105,360,106,361,107,362,363,108,109,364,110,365,112,111,113,366,368,114,367,369,115,370,371,116,117,372,118,373,119,120,374,375,121,376,122,377,123,378,379,124,125,380,126,381,127,382,128,383,129,384,130,385,131,386,132,387,133,388,389,134,135,390,391,136,137,392,138,393,139,394,140,395,141,396,142,397,143,398,144,145,399,400,146,401,147,402,148,403,149,404,150,405,151,406,407,152,153,408,154,409,155,410,411,156,157,412,158,413,159,414,160,415,161,416,162,417,163,418,164,419,165,420,166,421,167,422,423,168,169,424,170,425,171,426,172,427,173,428,174,429,175,430,431,176,177,432,178,433,179,434,180,435,181,436,182,437,183,438,439,184,185,440,186,441,187,442,188,443,189,444,192,190,445,193,191,448,194,446,449,195,447,450,451,196,197,452,198,453,199,454,455,200,201,456,202,457,203,458,459,204,205,460,206,461,207,462,208,463,209,464,210,465,211,466,467,212,213,468,214,469,215,470,471,216,217,472,218,473,219,474,475,220,221,476,222,477,224,223,225,478,480,226,479,481,227,482,483,228,229,484,230,485,231,486,487,232,233,488,234,489,235,490,491,236,237,492,238,493,239,240,494,241,496,495,242,497,243,498,499,244,245,500,246,501,247,502,503,248,249,504,250,505,251,506};

// LUT das fases
float PS_phase_deg_LUT[] = {0,0,0.4,0.7,1,1.6,1.9,2.7,2.8,2.8,3.6,3.7,4.1,4.5,4.9,5.5,5.5,6.1,6.5,7.4,8.4,8.4,9.4,10.4,11.4,11.6,12.7,13.7,14.5,14.9,15.7,16.7,17.8,17.9,19,19.9,21,21.2,22.3,23.1,23.4,24.3,24.4,25.2,26.1,26.2,27.1,28,29,29.1,30.1,31,32,32,33.1,34,35,35.3,36.4,37.4,38.3,38.5,39.4,40.4,41.6,41.7,42.9,43.8,44.8,45,46.1,47,47.4,48.2,48.3,49,49.8,50,50.8,51.6,52.4,52.6,53.4,54.2,55.1,55.2,56.2,56.9,58,58.1,59.2,60,60.8,61.1,61.9,62.8,63.8,63.9,65,65.7,66.6,67,67.9,68.7,69.9,70.6,71.5,72.3,73.1,73.3,74.1,74.9,75.8,75.9,76.8,77.6,78.5,78.6,79.5,80.4,81.5,81.5,82.6,83.5,84.3,84.6,85.4,86.3,87.4,87.4,88.5,89.4,89.9,90.3,90.5,90.8,91.5,91.8,92.3,92.3,92.5,93.1,93.5,94.2,95.1,95.3,96.2,97.3,97.9,98.2,98.8,99.9,100.9,101.4,101.9,103,103.7,104.1,104.6,105.8,106.9,107.2,108.4,109.5,110.2,110.7,111.4,112.5,112.9,113.7,113.7,114.8,115.4,115.8,116.2,117.3,118.2,118.4,119.4,120.4,121.1,121.4,122.1,123.2,124.2,124.2,125.2,126.4,126.9,127.4,127.9,129.3,130.3,130.7,131.8,133.1,133.8,134.2,134.7,134.9,135.6,136.2,136.6,137.1,137.3,137.4,137.9,139.1,140,140.4,141.3,142.4,143.1,143.4,144.1,145.3,146.3,146.4,147.4,148.7,149.3,149.8,150.4,151.7,152.8,153.3,154.4,155.7,156.5,156.8,157.6,157.7,158.4,158.9,159.6,160.1,160.2,160.4,161,162.2,163.2,163.6,164.5,165.7,166.5,166.7,167.4,167.5,168.7,169.7,171,172.1,172.9,173.2,174,175.2,176.4,177.1,178.2,179.4,180.4,180.6,181.5,182.7,183.8,184,184.8,185.4,186.4,186.4,187.5,188.1,189.2,189.2,190.3,190.9,192,192.1,193.2,193.9,195,195.5,196.7,197.4,198.5,198.6,199.7,200.4,201.6,201.7,203,203.7,204.8,205,206.1,206.8,207,207.9,208.2,208.8,209.7,209.8,210.7,211.6,212.6,212.6,213.6,214.4,215.5,215.5,216.6,217.4,218.6,219,220.2,221,222,222.2,223.2,224.1,225.3,225.4,226.6,227.4,228.5,228.7,229.8,230.6,231.5,232,232.5,233.1,233.9,234.1,235,235.7,236.4,236.7,237.5,238.1,239.1,239.2,240.3,240.9,242,242.3,243.4,244.1,245,245.3,246.2,246.9,247.9,248.2,249.2,249.8,250.8,251.1,252.1,252.8,254.1,254.7,255.7,256.4,257.3,257.4,258.3,259,259.8,260,260.9,261.6,262.6,262.6,263.7,264.4,265.5,265.8,266.9,267.7,268.5,268.8,269.7,270.5,271.6,271.7,272.8,273.5,274,274.5,274.7,274.8,275.7,275.7,276.3,276.4,276.6,277.2,277.7,278,279,279.1,280.1,281,281.7,282,282.7,283.6,284.6,284.8,285.9,286.8,287.5,287.9,288.6,289.6,290.7,291,292.2,293.2,294,294.4,295.2,296.2,297.1,297.4,297.9,298.8,299.4,299.7,300.3,301.3,302.2,302.4,303.4,304.3,305,305.3,306,307.1,308.1,308.2,309.2,310.3,310.9,311.4,312,313.2,314.3,314.6,315.7,316.8,317.6,318,318.5,318.8,319.3,319.9,320.3,320.9,321.1,321.2,321.8,322.8,323.7,324,325,326,326.8,326.9,327.8,328.8,329.8,330.1,331.2,332.2,333,333.4,334.1,335.3,336.4,336.8,338,338.9,340,340.3,341.2,341.7,342.3,342.4,343.5,343.6,344.2,344.4,345.1,346.1,347,347.4,348.4,349.4,350.2,350.4,351.2,352.3,353.3,353.7,354.7,355.8,356.6,357,357.7,358.9};

//////////////////////////////////////////////////////////////
//                FUNCTION PROTOTYPES
//////////////////////////////////////////////////////////////

void beamform(float theta_deg);
void nullSteer(float theta_null_deg, int m);
void phaseOnlyNullSteering(float theta_main_deg, float theta_null_deg);
void select_PS_bin();
void set_PS();
void setphase(int phase_bin, int address);
int findPhaseBin(float phase);
void manual_input_PS_bin();
void changeSpacing();
void previewBeam(float theta_deg);
void printMenu();
void flushSerial();
float to180(float deg);

//////////////////////////////////////////////////////////////
//                        SETUP
//////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);
    delay(1000);

    SPI.begin();
    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);

    SPI.setClockDivider(SPI_CLOCK_DIV64);
    SPI.setBitOrder(LSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    Serial.println("\n[INIT] Programming all channels to Bin 0 (Zero State)...");
    for (int idx = 0; idx < NUM_CHANNELS; idx++) {
        channel_phase_deg[idx] = 0.0;
        channel_phase_bin[idx] = PS_phase_bin_LUT[0]; // Bin 0
        setphase(PS_phase_bin_LUT[0], idx + ADDR_BASE);
        delay(5);
    }
    Serial.println("[INIT] System reset and ready!");

    flushSerial();
    printMenu();
}

//////////////////////////////////////////////////////////////
//                         LOOP
//////////////////////////////////////////////////////////////

void loop()
{
    if (millis() - last_rf_refresh >= REFRESH_INTERVAL_MS) {
        last_rf_refresh = millis();
        for (int idx = 0; idx < NUM_CHANNELS; idx++) {
            setphase(channel_phase_bin[idx], idx + ADDR_BASE);
        }
    }

    if (!Serial.available())
        return;

    char option = Serial.read();

    delay(10);
    while (Serial.available()) {
        Serial.read();
    }

    if (option == '\n' || option == '\r' || option == ' ' || option == '\0') {
        return;
    }

    switch (option)
    {
        case '1':
        case 'm':
        case 'M':
            manual_input_PS_bin();
            printMenu();
            break;

        case '2':
        case 'b':
        case 'B':
            Serial.println();
            Serial.print("Insert steering angle (deg): ");

            while (!Serial.available());
            steering_angle = Serial.parseFloat();
            flushSerial();
            Serial.print(steering_angle, 1);
            Serial.println(" deg");
            beamform(steering_angle);
            printMenu();
            break;

        case '3':
        case 's':
        case 'S':
            changeSpacing();
            printMenu();
            break;

        case '4':
        case 'p':
        case 'P':
            Serial.println();
            Serial.print("Insert steering angle to preview (deg): ");

            while (!Serial.available());
            steering_angle = Serial.parseFloat();
            flushSerial();
            Serial.print(steering_angle, 1);
            Serial.println(" deg");
            previewBeam(steering_angle);
            printMenu();
            break;

        case '5':
        case 'n':
        case 'N':
        {
            Serial.println();
            Serial.print("Insert null angle (deg): ");

            while (!Serial.available());
            float null_angle = Serial.parseFloat();
            flushSerial();
            Serial.print(null_angle, 1);
            Serial.println(" deg");

            Serial.print("Null order m (1-");
            Serial.print(ARRAY_SIZE - 1);
            Serial.print(", Enter = ");
            Serial.print(null_order_default);
            Serial.print("): ");

            while (!Serial.available());
            int m_in = Serial.parseInt();
            flushSerial();
            if (m_in == 0) m_in = null_order_default;
            Serial.println(m_in);

            nullSteer(null_angle, m_in);
            printMenu();
            break;
        }

        case '6':
        case 'i':
        case 'I':
        {
            Serial.println();
            Serial.print("Insert MAIN BEAM steering angle (deg): ");
            while (!Serial.available());
            float s_angle = Serial.parseFloat();
            flushSerial();
            Serial.print(s_angle, 1);
            Serial.println(" deg");

            Serial.print("Insert NULL angle (deg): ");
            while (!Serial.available());
            float n_angle = Serial.parseFloat();
            flushSerial();
            Serial.print(n_angle, 1);
            Serial.println(" deg");

            phaseOnlyNullSteering(s_angle, n_angle);
            printMenu();
            break;
        }

        default:
            Serial.print("\n[!] Invalid option ('");
            Serial.print(option);
            Serial.println("')");
            break;
    }
}

//////////////////////////////////////////////////////////////
//                    AUXILIARY FUNCTIONS
//////////////////////////////////////////////////////////////

void flushSerial()
{
    delay(20);
    while (Serial.available()) {
        Serial.read();
    }
}

//////////////////////////////////////////////////////////////
//          CONVERT 0-360 RANGE TO -180/+180
//////////////////////////////////////////////////////////////

float to180(float deg)
{
    if (deg > 180.0) deg -= 360.0;
    return deg;
}

//////////////////////////////////////////////////////////////
//                       MENU
//////////////////////////////////////////////////////////////

void printMenu()
{
    Serial.println();
    Serial.println("======================================");
    Serial.println("   5-Element Beamforming Controller   ");
    Serial.println("======================================");
    Serial.println("1 - Manual Phase Shifter");
    Serial.println("2 - Beam Steering");
    Serial.println("3 - Change Element Spacing");
    Serial.println("4 - Preview Beam");
    Serial.println("5 - Null Steering");
    Serial.println("6 - Independent Null Steering (Steyskal)");
    Serial.println("======================================");
    Serial.println("Select option [1, 2, 3, 4, 5, 6]:");
}

//////////////////////////////////////////////////////////////
//                    BEAMFORMING
//////////////////////////////////////////////////////////////

void beamform(float theta_deg)
{
    float theta = theta_deg * PI / 180.0;
    float delta_phase = -360.0 * element_spacing_lambda * sin(theta);

    Serial.println();
    Serial.println("----------- Beamforming -----------");
    Serial.print("Steering angle : ");
    Serial.print(theta_deg);
    Serial.println(" deg");
    Serial.print("Phase increment: ");
    Serial.print(delta_phase);
    Serial.println(" deg");
    
    Serial.println("\n--- Target Values ---");

    Serial.print("Element ");
    Serial.print(REF_ELEMENT_POS);
    Serial.print(" (ref) = ");
    Serial.print(to180(ref_phase), 2);
    Serial.println(" deg (Ref)");

    for (int idx = 0; idx < NUM_CHANNELS; idx++)
    {
        int physical_pos = PS_PHYSICAL_POS[idx];
        int offset_from_ref = physical_pos - REF_ELEMENT_POS;

        float phase = offset_from_ref * delta_phase;
        while (phase < 0) phase += 360.0;
        while (phase >= 360.0) phase -= 360.0;

        channel_phase_deg[idx] = phase;

        float absolute_phase = phase + ref_phase;
        while (absolute_phase < 0) absolute_phase += 360.0;
        while (absolute_phase >= 360.0) absolute_phase -= 360.0;

        Serial.print("Element ");
        Serial.print(physical_pos);
        Serial.print(" (Ch ");
        Serial.print(idx + ADDR_BASE);
        Serial.print(") | Theoretical = ");
        Serial.print(to180(phase), 1);
        Serial.print(" deg | Target = ");
        Serial.print(to180(absolute_phase), 2);
        Serial.println(" deg");
    }

    select_PS_bin();
}

//////////////////////////////////////////////////////////////
//                    NULL STEERING
//////////////////////////////////////////////////////////////

void nullSteer(float theta_null_deg, int m)
{
    if (m < 1 || m > (ARRAY_SIZE - 1)) {
        Serial.println("\n[!] Invalid null order (must be between 1 and ARRAY_SIZE-1).");
        m = 1;
        Serial.println("[!] Using m = 1 (null closest to main lobe).");
    }

    float theta_null = theta_null_deg * PI / 180.0;

    float phi = (360.0 * m / ARRAY_SIZE) - 360.0 * element_spacing_lambda * sin(theta_null);
    while (phi < 0) phi += 360.0;
    while (phi >= 360.0) phi -= 360.0;

    Serial.println();
    Serial.println("----------- Null Steering -----------");
    Serial.print("Target null      : ");
    Serial.print(theta_null_deg);
    Serial.println(" deg");
    Serial.print("Null order (m)   : ");
    Serial.println(m);
    Serial.print("Phase progression (phi): ");
    Serial.print(to180(phi));
    Serial.println(" deg");

    float sin_main = -to180(phi) / (360.0 * element_spacing_lambda);
    if (sin_main >= -1.0 && sin_main <= 1.0) {
        float theta_main = asin(sin_main) * 180.0 / PI;
        Serial.print("Resulting main lobe: ");
        Serial.print(theta_main, 1);
        Serial.println(" deg");
    } else {
        Serial.println("Resulting main lobe: out of visible space for this configuration.");
    }

    Serial.println("Other natural nulls for this configuration:");
    for (int mm = 1; mm <= ARRAY_SIZE - 1; mm++) {
        if (mm == m) continue;
        float sin_n = (360.0 * mm / ARRAY_SIZE - to180(phi)) / (360.0 * element_spacing_lambda);
        if (sin_n >= -1.0 && sin_n <= 1.0) {
            float theta_n = asin(sin_n) * 180.0 / PI;
            Serial.print("  m = ");
            Serial.print(mm);
            Serial.print(" -> ");
            Serial.print(theta_n, 1);
            Serial.println(" deg");
        }
    }

    Serial.println("\n--- Target Values (phase per element) ---");

    Serial.print("Element ");
    Serial.print(REF_ELEMENT_POS);
    Serial.print(" (ref) = ");
    Serial.print(to180(ref_phase), 2);
    Serial.println(" deg (Ref)");

    for (int idx = 0; idx < NUM_CHANNELS; idx++)
    {
        int physical_pos = PS_PHYSICAL_POS[idx];
        int offset_from_ref = physical_pos - REF_ELEMENT_POS;

        float phase = offset_from_ref * phi;
        while (phase < 0) phase += 360.0;
        while (phase >= 360.0) phase -= 360.0;

        channel_phase_deg[idx] = phase;

        float absolute_phase = phase + ref_phase;
        while (absolute_phase < 0) absolute_phase += 360.0;
        while (absolute_phase >= 360.0) absolute_phase -= 360.0;

        Serial.print("Element ");
        Serial.print(physical_pos);
        Serial.print(" (Ch ");
        Serial.print(idx + ADDR_BASE);
        Serial.print(") | Theoretical = ");
        Serial.print(to180(phase), 1);
        Serial.print(" deg | Target = ");
        Serial.print(to180(absolute_phase), 2);
        Serial.println(" deg");
    }

    select_PS_bin();
}

//////////////////////////////////////////////////////////////
//          INDEPENDENT PHASE-ONLY NULL STEERING
//////////////////////////////////////////////////////////////

void phaseOnlyNullSteering(float theta_main_deg, float theta_null_deg)
{
    float theta_m = theta_main_deg * PI / 180.0;
    float theta_n = theta_null_deg * PI / 180.0;

    float psi = 2.0 * PI * element_spacing_lambda * (sin(theta_n) - sin(theta_m));

    float F0 = 1.0 + 2.0 * cos(psi) + 2.0 * cos(2.0 * psi);

    float den = 2.0 * (pow(sin(psi), 2) + pow(sin(2.0 * psi), 2));
    
    if (den < 0.001) {
        Serial.println("\n[!] Error: Impossible to place null so close to the main beam.");
        return;
    }
    
    float c = F0 / den;

    float delta_deg[ARRAY_SIZE]; 
    
    delta_deg[0] = (-c * sin(2.0 * psi)) * (180.0 / PI); 
    delta_deg[1] = (-c * sin(psi))       * (180.0 / PI); 
    delta_deg[2] = 0.0;                                  
    delta_deg[3] = (c * sin(psi))        * (180.0 / PI); 
    delta_deg[4] = (c * sin(2.0 * psi))  * (180.0 / PI); 

    Serial.println();
    Serial.println("------- Independent Null Steering -------");
    Serial.print("Main Lobe      : "); Serial.print(theta_main_deg); Serial.println(" deg");
    Serial.print("Target Null    : "); Serial.print(theta_null_deg); Serial.println(" deg");

    float delta_phase_main = -360.0 * element_spacing_lambda * sin(theta_m);

    Serial.println("\n--- Target Values (phase per element) ---");
    Serial.print("Element ");
    Serial.print(REF_ELEMENT_POS);
    Serial.print(" (ref) = ");
    Serial.print(to180(ref_phase), 2);
    Serial.println(" deg (Ref) -> [Perturbation: 0.0 deg]");

    for (int idx = 0; idx < NUM_CHANNELS; idx++)
    {
        int physical_pos = PS_PHYSICAL_POS[idx];
        int n = physical_pos - REF_ELEMENT_POS; 
        
        float phase_main = n * delta_phase_main;
        
        float phase = phase_main + delta_deg[physical_pos];
        
        while (phase < 0) phase += 360.0;
        while (phase >= 360.0) phase -= 360.0;

        channel_phase_deg[idx] = phase;

        float absolute_phase = phase + ref_phase;
        while (absolute_phase < 0) absolute_phase += 360.0;
        while (absolute_phase >= 360.0) absolute_phase -= 360.0;

        Serial.print("Element "); Serial.print(physical_pos);
        Serial.print(" (Ch "); Serial.print(idx + ADDR_BASE);
        Serial.print(") | Perturbation: "); Serial.print(delta_deg[physical_pos], 1);
        Serial.print(" deg | Theoretical = "); Serial.print(to180(phase), 1);
        Serial.print(" deg | Target = "); Serial.print(to180(absolute_phase), 2);
        Serial.println(" deg");
    }

    select_PS_bin();
}

//////////////////////////////////////////////////////////////
//                PREVIEW BEAM
//////////////////////////////////////////////////////////////

void previewBeam(float theta_deg)
{
    float theta = theta_deg * PI / 180.0;
    float delta_phase = -360.0 * element_spacing_lambda * sin(theta);

    Serial.println();
    Serial.println("----------- Preview -----------");
    Serial.print("Steering angle : ");
    Serial.print(theta_deg);
    Serial.println(" deg");
    Serial.print("Spacing        : ");
    Serial.print(element_spacing_lambda, 3);
    Serial.println(" lambda");
    Serial.print("Phase increment: ");
    Serial.print(delta_phase);
    Serial.println(" deg");
    Serial.println();

    Serial.println("--- Target Values ---");

    for (int pos = 0; pos < ARRAY_SIZE; pos++)
    {
        int offset_from_ref = pos - REF_ELEMENT_POS;
        
        float phase = offset_from_ref * delta_phase;
        while (phase < 0) phase += 360.0;
        while (phase >= 360.0) phase -= 360.0;

        float absolute_phase = phase + ref_phase;
        while (absolute_phase < 0) absolute_phase += 360.0;
        while (absolute_phase >= 360.0) absolute_phase -= 360.0;

        Serial.print("Element ");
        Serial.print(pos);
        if (pos == REF_ELEMENT_POS) {
            Serial.print(" (Ref)");
        } else {
            int addr = -1;
            for (int idx = 0; idx < NUM_CHANNELS; idx++) {
                if (PS_PHYSICAL_POS[idx] == pos) {
                    addr = idx + ADDR_BASE;
                    break;
                }
            }
            Serial.print(" (Ch ");
            Serial.print(addr);
            Serial.print(")");
        }
        Serial.print(" | Target = ");
        Serial.print(to180(absolute_phase), 2);
        Serial.println(" deg");
    }

    Serial.println("------------------------------------------");
    Serial.println("(No data sent to hardware.)");
}

//////////////////////////////////////////////////////////////
//                BINARY SEARCH LUT
//////////////////////////////////////////////////////////////

int findPhaseBin(float phase)
{
    int low = 0;
    int high = 511;

    while (low <= high)
    {
        int mid = (low + high) / 2;
        if (PS_phase_deg_LUT[mid] < phase)
            low = mid + 1;
        else
            high = mid - 1;
    }

    if (low <= 0) return PS_phase_bin_LUT[0];
    if (low >= 512) return PS_phase_bin_LUT[511];

    float errLower = fabs(phase - PS_phase_deg_LUT[low - 1]);
    float errUpper = fabs(PS_phase_deg_LUT[low] - phase);

    if (errLower <= errUpper)
        return PS_phase_bin_LUT[low - 1];
    else
        return PS_phase_bin_LUT[low];
}

//////////////////////////////////////////////////////////////
//          CONVERT PHASE TO PHASE SHIFTER BIN
//////////////////////////////////////////////////////////////

void select_PS_bin()
{
    Serial.println("\nCompensating phase offsets...");

    for (int idx = 0; idx < NUM_CHANNELS; idx++)
        channel_phase_bin[idx] = 1000;

    for (int idx = 0; idx < NUM_CHANNELS; idx++)
    {
        float phase = channel_phase_deg[idx] - channel_phase_offset[idx];
        while (phase < 0) phase += 360.0;
        while (phase >= 360.0) phase -= 360.0;

        channel_phase_bin[idx] = findPhaseBin(phase);

        Serial.print("Channel ");
        Serial.print(idx + ADDR_BASE);
        Serial.print(" | Comp. Mat. = ");
        Serial.print(to180(phase), 1);
        Serial.print(" deg | Bin = ");
        Serial.println(channel_phase_bin[idx]);
    }

    set_PS();
}

//////////////////////////////////////////////////////////////
//          SEND DATA TO PHASE SHIFTER
//////////////////////////////////////////////////////////////

void setphase(int phase_bin, int address)
{
    sendphase = address;
    sendphase <<= 9;
    sendphase += phase_bin;
    sendphase <<= 3;

    digitalWrite(SS, LOW);
    SPI.transfer16(sendphase);
    digitalWrite(SS, HIGH);
}

//////////////////////////////////////////////////////////////
//          PROGRAM ALL FOUR PHASE SHIFTERS
//////////////////////////////////////////////////////////////

void set_PS()
{
    Serial.println("\nProgramming phase shifters...");

    for (int idx = 0; idx < NUM_CHANNELS; idx++)
    {
        setphase(channel_phase_bin[idx], idx + ADDR_BASE);
        Serial.print("Channel ");
        Serial.print(idx + ADDR_BASE);
        Serial.print(" -> Bin ");
        Serial.println(channel_phase_bin[idx]);
        delay(5);
    }

    Serial.println("\nBeam successfully updated.");
}

//////////////////////////////////////////////////////////////
//                MANUAL TEST MODE
//////////////////////////////////////////////////////////////

void manual_input_PS_bin()
{
    int address;
    int bin;

    Serial.println();
    Serial.println("---------- Manual Mode ----------");
    Serial.print("Address (2-5): ");

    while (!Serial.available());
    address = Serial.parseInt();
    flushSerial();

    if (address < ADDR_BASE || address >= ADDR_BASE + NUM_CHANNELS)
    {
        Serial.println("\nInvalid address.");
        return;
    }

    Serial.print(address);
    Serial.println();
    Serial.print("Phase-shifter bin (0-511): ");

    while (!Serial.available());
    bin = Serial.parseInt();
    flushSerial();

    if (bin < 0 || bin > 511)
    {
        Serial.println("\nInvalid bin.");
        return;
    }

    Serial.print(bin);
    Serial.println();
    Serial.print("Programming Channel ");
    Serial.print(address);
    Serial.print(" with Bin ");
    Serial.println(bin);

    setphase(bin, address);

    int idx = address - ADDR_BASE;
    channel_phase_bin[idx] = bin;

    Serial.println("Done.");
}

//////////////////////////////////////////////////////////////
//            CHANGE ELEMENT SPACING
//////////////////////////////////////////////////////////////

void changeSpacing()
{
    Serial.println();
    Serial.print("Current spacing = ");
    Serial.print(element_spacing_lambda, 3);
    Serial.println(" lambda");
    Serial.print("New spacing: ");

    while (!Serial.available());
    float value = Serial.parseFloat();
    flushSerial();
    Serial.print(value, 3);
    Serial.println(" lambda");

    if (value <= 0.0 || value > 1.0)
    {
        Serial.println("\nInvalid spacing (must be > 0 and <= 1.0).");
        return;
    }

    element_spacing_lambda = value;
}