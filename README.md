# washing_machine
Půlsemestrální projekt embedded systémů - podel pračky jako konečný automat

Zadání
Vytvořte program s využitím modelu pračky připojeného k výukovému kitu.
Definujte si sami zadání pro váš model. Například model pračky může představovat automatickou pračku.
Program bude splňovat následující:
• Budou využity pokud možno všechny vstupy a výstupy modelu.
• Program bude mít sekvenční chování. Např. u modelu pračky bude prací program, který
postupně napouští vodu, ohřívá ji, točí bubnem atd.
• Program bude realizován jako konečný automat, viz poznámka 1 níže.
• Pro ovládání programu bude použito kromě tlačítek také potenciometru na výukovém kitu,
např. nastavování teploty pro pračku, doby míchání pro mísicí jednotku.
• Průběh programu (praní, přípravy směsi apod.) by mělo být možno přerušit stiskem tlačítka.
• Kód programu by měl být srozumitelný, strukturovaný do funkcí a komentovaný.
Doporučeno je rozdělení na úlohy (tasky) jak je ukázáno ve cvičení 4.
• Pro model pračky bude vytvořen ovladač v samostatném .c a .h souboru

Rozhraní ovladače pro model pračky
Ovladač pro model pračky by měl obsahovat níže uvedené funkce.
Názvy funkcí a jejich parametry můžete upravit podle uvážení. Například místo funkce NastavNapousteni
s parametrem zda zapnout nebo vypnout můžete vytvořit 2 funkce: ZapniNapousteni() a VypniNapousteni().
Funkce ovladače:
NastavNapousteni( zapnout/vypnout )
NastavTopeni( zapnout/vypnout )
NastavBuben( směr vlevo-vpravo, zvýšené otáčky ano/ne )
NastavCerpadlo( zapnout/vypnout )
int CtiTeplotu() – vrací hodnotu teploty vody v pračce.
int CtiHladinu() – vrací úroveň vody v pračce.
