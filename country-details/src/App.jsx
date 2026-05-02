import { Suspense } from 'react'
import './App.css'
import Countries from './components/countries/Countries'



// Get Country File ALL
const getCountryAll = async() =>{
const res = await fetch('https://restcountries.com/v3.1/all')
return(res.json())
} 
const AllCountries = getCountryAll();

function App() {
  return (
    <>

<Suspense fallback={<h2 className='text-center text-4xl mt-40'>Loading Data File  <span><span className="loading loading-spinner loading-xl"></span></span></h2>}>
  <Countries AllCountries={AllCountries}></Countries>
</Suspense>

    </>
  )
}

export default App
