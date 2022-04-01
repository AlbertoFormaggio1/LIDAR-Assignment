#include <fstream>
#include <random>
#include <ctime>
#include <string>
#include "LaserScannerDriver.h"
#define _CRTDBG_MAP_ALLOC

using namespace std;

bool fill(string file_name, vector<double>& v);
LaserScannerDriver test_copy(const LaserScannerDriver& lsd, bool copy_and_test);
void test_contructor_assignment(const LaserScannerDriver& first, const LaserScannerDriver& other);

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	srand(static_cast<int>(time(nullptr)));	//Setto il seed del generatore random per dopo

	LaserScannerDriver lsd1(1);
	cout << "Creazione con risoluzione " << lsd1.angular_resolution() << " ok" << endl;
	LaserScannerDriver lsd2(0.4);
	cout << "Creazione con risoluzione " << lsd2.angular_resolution() << " ok" << endl;
	LaserScannerDriver lsd3(0.1);
	cout << "Creazione con risoluzione " << lsd3.angular_resolution() << " ok" << endl;
	try
	{
		LaserScannerDriver lsd4(-0.2);
		cout << "Creazione con risoluzione " << lsd4.angular_resolution() << "ok" << endl;
	}
	catch (out_of_range e)
	{
		cerr << e.what();
	}

	cout << endl << endl;

	//Riempio il vector con i dati
	vector<double> v1;
	vector<double> v2;
	vector<double> v3;
	if (!fill("input1.txt", v1))
		return -1;
	if (!fill("input2.txt", v2))
		return -1;
	if (!fill("input3.txt", v3))
		return -1;
	cout << "File reading ok" << endl;
	cout << endl;

	//Nota per il debug successivo: il primo numero di ogni scansione è il numero del file e del vettore, per comodità (v1[0] = primo valore di input1.txt = 1)


	/*************TESTING DI NEW_SCAN() E GET_DISTANCE()*************/

	//Inserisco diversi vector
	cout << "testing new_scan() and get_distance():" << endl;
	LaserScannerDriver lsd(0.764);
	//Temporarly store first values of each scan
	constexpr int scans = 3;
	double dist[scans];
	lsd.new_scan(v1);
	dist[0] = lsd.get_distance(0);
	lsd.new_scan(v2);
	dist[1] = lsd.get_distance(0);
	lsd.new_scan(v3);
	dist[2] = lsd.get_distance(0);

	//Controllo se la scan i-esima ha come primo valore proprio i
	for (int i = 0; i < scans; i++)
	{
		cout << "Scan " << i + 1 << " ";
		if (dist[i] == static_cast<double>(i + 1))	//Cast non necessario in quanto eseguito implicitamente, scritto comunque esplicitamente solo per chiarezza
			cout << "ok";
		else
			cout << "error";
		cout << endl;
	}
	cout << endl;

	/*************TESTING DI GET_SCAN()*************/

	//Rimuovo v2 (test fatto con dimensione di buffer_dim = 2, quindi v1 è stato sovrascritto da v3. Con altri test, sovrascrivere la variabile costante vector_number dopo aver fatto i calcoli)
	constexpr int vector_number = 2;
	cout << "Testing get_scan(): this should be vector" << vector_number << endl;

	vector<double> removed = lsd.get_scan();
	double first_value = removed[0];
	cout << "This is vector" << first_value << endl;

	if (vector_number == static_cast<int>(first_value))
		cout << "get_scan() ok";
	else
		cout << "get_scan() error";

	cout << endl << endl;

	/*************TESTING DI COSTRUTTORE COPY E MOVE*************/

	//Dentro metodo test_copy() si usa il copy constructor, al ritorno dal metodo verrà invocato il move constructor per assegnare l'rvalue temporaneo ritornato
	cout << "Testing copy constructor: " << endl;;
	const LaserScannerDriver move_lsd = test_copy(lsd, true); //Sarà possibile usare solo i metodi get_angular_resolution() e get_distance()

	cout << endl;

	cout << "Testing move constructor: " << endl;
	test_contructor_assignment(lsd, move_lsd);
	
	cout << endl;


	/*************TESTING DI ASSEGNAMENTO COPY E MOVE*************/

	cout << "Testing copy assignment: " << endl;
	LaserScannerDriver copy_lsd;
	copy_lsd.new_scan(v1);
	copy_lsd.new_scan(v2);
	copy_lsd = lsd; //Copy assignment
	test_contructor_assignment(lsd, copy_lsd);

	cout << endl;

	cout << "Testing move assigment: " << endl;
	LaserScannerDriver move_lsd1;
	move_lsd1.new_scan(v1);
	move_lsd1.new_scan(v3);
	move_lsd1.new_scan(v2);
	move_lsd1 = test_copy(copy_lsd, false); //Move assignment
	test_contructor_assignment(copy_lsd, move_lsd1);

	cout << endl;


	/*************TESTING DI CLEAR_BUFFER()*************/

	cout << "clearing buffer..." << endl;
	lsd.clear_buffer();
	cout << lsd;

	//ora invocare get_scan() dovrebbe dare problemi se lo scanner è vuoto
	try
	{
		lsd.get_scan();
		cout << "get_scan() successful. Test gone wrong";
	}
	catch (LaserScannerDriver::EmptyBufferException)
	{
		cout << "buffer is empty! Test is successful"; 
	}

	cout << endl << endl;


	/*************TESTING COUT E MOVE E COPY ASSIGNMENT*************/

	//Ora lsd è vuoto. Se move_lsd è stato fatto correttamente (cioè con una deep copy), dentro dovrebbero ancora esserci dei valori. Così come dentro copy_lsd, li stampo entrambi a schermo
	cout << "Testing cout and deep copy effectiveness: " << endl;
	cout << "move_lsd:" << endl << move_lsd << endl << endl;;
	cout << "copy_lsd:" << endl << copy_lsd << endl;

}


/*!
 * @brief Fa una copia dell'oggetto passato e la ritorna così da testare: copy, move constructor e move assignment
 * @param copy_and_test vero se deve fare anche il test, falso altrimenti
*/
LaserScannerDriver test_copy(const LaserScannerDriver& lsd, bool copy_and_test)
{
	cout << "calling copy constuctor..." << endl;
	LaserScannerDriver copy_lsd = lsd;

	if(copy_and_test)
		test_contructor_assignment(lsd, copy_lsd);

	return copy_lsd;
}

/*!
 * @brief Controlla se la copia dei dati ha avuto successo
 * @param first Oggetto originale
 * @param other Oggetto copiato
*/
void test_contructor_assignment(const LaserScannerDriver& first, const LaserScannerDriver& other)
{
	//Faccio un controllo solo su un valore (casuale) per evitare di inviare troppi valori in output che mi impediscono di vedere tutto l'output generato dal main (da VisualStudio almeno
	//la finestra della console generata può contenere un numero massimo di righe) 

	const int angle = rand() % (static_cast<int>(LaserScannerDriver::kMaxAngle + 1)); //Genero un numero casuale tra 0 e 180 (estremi inclusi)
	double d1 = first.get_distance(angle);											  //Ritorno la distanza relativa a quell'angolo
	double d2 = other.get_distance(angle);

	cout << "original data: resolution = " << first.angular_resolution() << "; get_distance(" << angle << ") = " << d1 << endl;
	cout << "copied/moved data: resolution = " << other.angular_resolution() << "; get_distance(" << angle << ") = " << d2 << endl;

	if (first.angular_resolution() == other.angular_resolution())
		cout << "angular resolution ok";
	else
		cout << "error copying angular resolution";

	cout << endl;

	if (d1 == d2)
		cout << "value ok";
	else
		cout << "error while copying values";

	cout << endl;
}

/*!
 * @brief Riempie il vector v passato per reference con i valori inclusi nel file fornito
 * @return vero se la copia dei valori ha avuto successo, falso altrimenti
*/
bool fill(string file_name, vector<double>& v)
{
	ifstream my_file;

	my_file.open(file_name);
	if (my_file.fail())	//Se il file non è stato trovato ritorna falso
	{
		cout << "File not found";
		return false;
	}

	string read_value;
	while (!my_file.eof())	//Leggo finchè non raggiungo l'end of file
	{
		my_file >> read_value;
		try
		{
			v.push_back(stod(read_value));
		}
		catch (exception e)
		{
			//Eccezione non gestita volutamente: se trovo qualcosa che non sia un numero, lo ignoro
		}
	}

	//Libero i file aperti
	my_file.close();

	return true;
}