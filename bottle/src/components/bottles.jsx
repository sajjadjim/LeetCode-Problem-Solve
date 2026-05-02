import React, { use, useEffect, useState } from 'react';
import Bottle from './bottle/bottle';
import { addStoredCart , getItems , removeCart } from './utilities/localStorage';
import './bottles.css'
import Cart from './cart/cart';
export default function Bottles ({bottlePromise}){
 const bottles = use(bottlePromise)
//  console.log(Bottles) 
const [cartBottles , setCartBottle] = useState([])

// useEffect to get value show here 
useEffect(()=>{
    const getCartItem = getItems();
    const storeCartBottle = []
    for(let id of getCartItem){
        const cartBottle = bottles.find(bottles => bottles.id === id)

        if(cartBottle){ 
           storeCartBottle.push(cartBottle);
        }
    }
// Now here setCart Value set 
setCartBottle(storeCartBottle)
},[bottles])   

const handleCartBottle = (bottle_single) =>{
const newCartBottle = [...cartBottles , bottle_single ]
setCartBottle(newCartBottle)
// console.log('Cart Bottle Add',bottle_single)

// Save the Data Local storage 
addStoredCart(bottle_single.id)
}

// Remove the Cart Items From Cart 
const handleCartItems = (id) =>{
    // console.log('Delete From Cart Item' ,id)
    const remainingCart = cartBottles.filter(bottles => bottles.id !== id )
    setCartBottle(remainingCart)
    removeCart(id)
}

    return (
        <div>
  <h1 style={{textAlign:'center'}}>Total Number of Bottle : {bottles.length}</h1>
  <h3>Add cart :{cartBottles.length}</h3>

  <Cart 
  cartBottles={cartBottles}
  handleCartItems={handleCartItems}
  ></Cart>  

  <div className=' grid md:grid-cols-3 grid-cols-1'>
  {
    bottles.map(bottle_single=><Bottle 
      key={bottle_single.id} 
      handleCartBottle={handleCartBottle}
      bottle_single={bottle_single}>
      </Bottle>)
  }
  </div>
        </div>
    );
};
