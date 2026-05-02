// # Steps we can use the Data Store in Local File 
const getFromLocalStorage = () =>{
    const storedString =localStorage.getItem('unique_id')

    if(storedString){
        const storedCountryName = JSON.parse(storedString)
        return storedCountryName;
    }
    return []
}

const saveCountryNameLocalStorage = (unique_id) =>{
const countryNameStringConverted = JSON.stringify(unique_id)
localStorage.setItem(unique_id,countryNameStringConverted)
}

const addCountryNameLocalStorage = (cca3) =>{
    const unique_id = getFromLocalStorage()
    const newCountryName = [...unique_id , cca3]
    saveCountryNameLocalStorage(newCountryName)
}
export {addCountryNameLocalStorage as addCountry , getFromLocalStorage as getCountry}